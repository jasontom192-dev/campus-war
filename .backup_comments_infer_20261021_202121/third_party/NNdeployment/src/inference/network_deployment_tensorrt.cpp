#include "network_deployment_tensorrt.hpp"

#if NNDEPLOYMENT_WITH_TENSORRT

// ============================================================================
// network_deployment_tensorrt.cpp —— ★ 本工程实际执行的推理代码
//
// 三段职责：
//   构造：读 .engine 二进制 → createInferRuntime → deserializeCudaEngine →
//         createExecutionContext → 读输入输出张量形状 → 分配显存/页锁定内存/流
//   infer：blobFromImage 做【颜色+数值+排布】转换 → 拷到显存 → enqueueV3 →
//         拷回页锁定内存 → 返回裸指针
//   析构：逆序释放 CUDA 资源
//
// ★★ 本工程"输入是 f32 / RGB / 0~1 / NCHW"这个契约的落点就在下面的
//    cv::dnn::blobFromImage(图, blob, 1/255.0, size, Scalar(0,0,0),
//                           /*swapRB=*/true, /*crop=*/false, CV_32F)：
//      1/255.0   → 归一化到 0~1
//      swapRB    → BGR 输入转成模型要的 RGB（★ 这一条最关键，
//                  预处理层刻意不转颜色，全靠这里）
//      CV_32F    → f32
//      输出布局  → blobFromImage 恒为 NCHW
//   四个条件缺一不可，改任何一个都要同步确认模型那边的导出设置。
//
// ★ 这份代码的 CUDA 错误【一律不检查】：cudaMalloc / cudaMemcpyAsync /
//   enqueueV3 的返回值都没看。出问题时表现为结果全 0 或崩溃，而不是报错。
//   上游如此，本工程不改；真要排查可以在这些调用后加 cudaGetLastError。
// ============================================================================

#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

namespace
{
// 打印模型输入输出张量的名字/方向/形状/数据类型。
// 只在 DebugConfig::print_debug_info 打开时调用。
// ★ 排查"模型和代码对不上"时最先看这段输出：本工程期望的是
//   输入 images [1,3,480,640] float32（注意 TRT 报的是 NCHW 顺序，
//   和代码里的 640×480 是转置关系），输出 output0 [1,21,6300] float32。
// ★ 它按索引 0/1 取张量名，不依赖 "images"/"output0" 这两个硬编码名字，
//   所以哪怕名字被改了这段也能打印出来。
void printTensorInfo(const nvinfer1::ICudaEngine *engine)
{
    if (!engine)
    {
        std::cerr << "错误: 引擎未初始化" << std::endl;
        return;
    }

    std::cout << "=== 模型张量信息 ===" << '\n';
    for (int i = 0; i < 2; ++i)
    {
        const char *name = engine->getIOTensorName(i);
        nvinfer1::TensorIOMode ioMode = engine->getTensorIOMode(name);
        nvinfer1::Dims dims = engine->getTensorShape(name);
        nvinfer1::DataType dtype = engine->getTensorDataType(name);

        std::cout << "张量 " << i << ":" << '\n';
        std::cout << "  名称: " << name << '\n';
        std::cout << "  类型: " << (ioMode == nvinfer1::TensorIOMode::kINPUT ? "输入" : "输出") << '\n';
        std::cout << "  形状: [";
        for (int j = 0; j < dims.nbDims; ++j)
        {
            std::cout << dims.d[j];
            if (j < dims.nbDims - 1)
                std::cout << ", ";
        }
        std::cout << "]" << '\n';

        std::cout << "  数据类型: ";
        switch (dtype)
        {
        case nvinfer1::DataType::kFLOAT:
            std::cout << "float32";
            break;
        case nvinfer1::DataType::kHALF:
            std::cout << "float16";
            break;
        case nvinfer1::DataType::kINT8:
            std::cout << "int8";
            break;
        case nvinfer1::DataType::kINT32:
            std::cout << "int32";
            break;
        case nvinfer1::DataType::kBOOL:
            std::cout << "bool";
            break;
        default:
            std::cout << "unknown";
        }
        std::cout << '\n' << '\n';
    }
    std::cout.flush();
}
} // namespace

// 构造：整个流程都是"一步失败就抛 runtime_error"，没有中间态。
// ★ model_path 在这里必须是【序列化好的 .engine 文件】，不是 .onnx；
//   而且 engine 和 GPU 架构、TensorRT 版本强绑定，换机器/升级 TRT 之后
//   反序列化会失败（报错信息通常很难懂，重导一次 engine 是标准解法）。
TensorRTEngine::TensorRTEngine(const YOLOModel::ModelConfig &model_config, const DebugConfig &debug_config) : InferenceEngine(model_config, debug_config)
{
    // ==================== 初始化推理引擎 ====================
    // 读取引擎文件
    // ★ 读不到就抛，异常会一路传到 ArmorModel 的构造函数外面。
    //   所以 Detector 构造失败最常见的两个原因就是：模型路径配错、engine 过期。
    std::ifstream engine_file(m_model_config.model_path, std::ios::binary);
    if (!engine_file)
        throw std::runtime_error("找不到 TensorRT 模型: " + m_model_config.model_path);

    // 获取文件大小
    engine_file.seekg(0, std::ios::end);
    const size_t file_size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);

    // 以二进制文件形式加载模型内容
    std::vector<char> engine_data(file_size);
    if (!engine_file.read(engine_data.data(), file_size))
        throw std::runtime_error("读取 TensorRT 模型失败: " + m_model_config.model_path);

    engine_file.close();

    // 创建TensorRT运行时对象
    m_runtime.reset(nvinfer1::createInferRuntime(m_logger));
    if (!m_runtime)
        throw std::runtime_error("无法创建 TensorRT 运行时");

    // 反序列化引擎数据,生成推理引擎
    m_engine.reset(m_runtime->deserializeCudaEngine(engine_data.data(), file_size));
    if (!m_engine)
        throw std::runtime_error("无法反序列化 TensorRT 模型: " + m_model_config.model_path);

    // 创建执行上下文
    m_context.reset(m_engine->createExecutionContext());
    if (!m_context)
        throw std::runtime_error("无法创建 TensorRT 执行上下文");

    // 写入输入输出张量大小
    // ★ 这一步就是"形状驱动"的源头：输入尺寸从 engine 读（本工程 d[3]=640 宽、
    //   d[2]=480 高，即输入张量是 [1,3,480,640] NCHW）；
    //   下面两行把输出张量读成 [rows, cols]，auto_detect 就是靠这两个值
    //   反推出该用 V8_21InfantryPostProcessor 的（本工程 [21, 6300]）。
    //   所以换 engine 等于自动换后处理，不需要改任何代码。
    m_target_width = m_engine->getTensorShape(m_engine->getIOTensorName(m_input_index)).d[3];
    m_target_height = m_engine->getTensorShape(m_engine->getIOTensorName(m_input_index)).d[2];

    // ★ 取的是 d[2]（本工程 = 6300，即"候选数"），但这个成员在库外没人用，
    //   是上游遗留字段。真正被后处理用的是下面的 out_tensor_cols。
    m_output_anchors = m_engine->getTensorShape(m_engine->getIOTensorName(m_output_index)).d[2];

    m_infer_param.out_tensor_rows = m_engine->getTensorShape(m_engine->getIOTensorName(m_output_index)).d[1];
    m_infer_param.out_tensor_cols = m_engine->getTensorShape(m_engine->getIOTensorName(m_output_index)).d[2];

    m_input_volume = 1 * sizeof(float);
    for (int i = 0; i < m_engine->getTensorShape(m_engine->getIOTensorName(m_input_index)).nbDims; i++)
    {
        m_input_volume *= m_engine->getTensorShape(m_engine->getIOTensorName(m_input_index)).d[i];
    }
    m_output_volume = 1 * sizeof(float);
    for (int i = 0; i < m_engine->getTensorShape(m_engine->getIOTensorName(m_output_index)).nbDims; i++)
    {
        m_output_volume *= m_engine->getTensorShape(m_engine->getIOTensorName(m_output_index)).d[i];
    }

    // ==================== 分配推理缓冲区 ====================
    // 获取输入张量的形状并设置
    // ★ 张量名 "images"/"output0" 是写死的 —— 这两个名字来自模型的
    //   ONNX 导出脚本。换模型时名字对不上，TRT 会在 setInputShape/
    //   setTensorAddress 上报错（或静默绑不上），是换 engine 后第二常见的坑
    //   （第一常见是 engine 和当前 GPU/TRT 版本不匹配）。
    // ★ 这里是静态形状引擎的标准做法；动态形状的话需要显式 setInputShape，
    //   本工程模型是固定 640×480，所以这行等于重设成原值。
    nvinfer1::Dims inputDims = m_engine->getTensorShape("images");
    m_context->setInputShape("images", inputDims);

    // 分配GPU显存用于输入输出
    // ★ m_input_volume / m_output_volume 的单位是【字节】（前面算的时候已经乘过
    //   sizeof(float)），所以这里直接当字节数用是对的，别再乘元素大小。
    cudaMalloc(&m_buffers[m_input_index], m_input_volume);
    cudaMalloc(&m_buffers[m_output_index], m_output_volume);

    // 绑定显存地址到TensorRT上下文
    m_context->setTensorAddress("images", m_buffers[m_input_index]);
    m_context->setTensorAddress("output0", m_buffers[m_output_index]);

    // 分配页锁定内存用于存储输入输出数据（提高传输效率）
    // ★ 页锁定（pinned）内存能让 cudaMemcpyAsync 真正异步、并绕过一次内部拷贝；
    //   代价是这块内存不会被换出，分配多了会拖慢整机。
    // m_rst 就是后处理拿到的那个裸指针的宿主，大小 = 529200 字节（21*6300*4）。
    cudaMallocHost(reinterpret_cast<void **>(&m_rst), m_output_volume);
    cudaMallocHost(&m_blob_pinned, m_input_volume);

    // 创建CUDA流用于异步操作
    // ★ 整个引擎共用这一条流，所以这个类不可重入（多线程同时 infer 会串数据）。
    cudaStreamCreate(&m_cuda_stream);

    if (m_debug_config.print_debug_info)
        printTensorInfo(m_engine.get());
}

int TensorRTEngine::inputWidth() const
{
    return m_target_width;
}

int TensorRTEngine::inputHeight() const
{
    return m_target_height;
}

// 同步推理：本工程每帧调用的就是这个（infer_mode 写 "sync"）。
//
// 输入：pre_processed_image —— 预处理层给的 u8 BGR Mat，尺寸已经等于
//       m_target_width × m_target_height（本工程 640×480）。
// 输出：m_rst —— f32 裸指针，内容即 [1, 21, 6300]，直接交给后处理。
//       ★ 它的形状信息不在这里，后处理靠 InferParam 里的 rows/cols 解释它。
//
// ★ blobFromImage 的九个参数逐个说清楚（这是全库最"有信息量"的一行）：
//     输入图、输出 blob、scaleFactor = 1/255.0  → 归一化到 0~1
//     size = 目标尺寸                           → ★ 又 resize 了一次
//     mean = Scalar(0,0,0)                      → 不减均值
//     swapRB = true                             → BGR 转 RGB（模型要 RGB）
//     crop  = false                             → 不做中心裁剪
//     ddepth = CV_32F                           → f32；输出恒为 NCHW
//   注意 size 这次 resize 是【多余动作】：进到这里时图已经是 640×480 了，
//   所以它实际是一次等比的无操作缩放（但会多一次内存拷贝）。保留是上游写法，
//   好处是"这个函数单独看是自洽的"。
//
// ★ 内存路径：m_blob(普通内存) --memcpy--> m_blob_pinned(页锁定)
//   --cudaMemcpyAsync--> 显存 --enqueueV3--> 显存 --cudaMemcpyAsync--> m_rst
//   之所以多一次 memcpy，是因为直接把 blobFromImage 的输出当异步源不可靠
//   （普通内存不是页锁定的，异步拷贝可能读到还没写好的数据）。
//
// ★ 结尾的 cudaStreamSynchronize 让它变成彻底的同步调用：函数返回时结果已经
//   在 m_rst 里，可以立刻做后处理。这就是"sync 和 async 一样"的根源。
const float *TensorRTEngine::syncInfer(const cv::Mat &pre_processed_image)
{
    // BGR(u8,HWC) → RGB(f32,0~1,NCHW)，模型输入的最终形态在这里成型
    cv::dnn::blobFromImage(pre_processed_image, m_blob, 1 / 255.0,
                           cv::Size(m_target_width, m_target_height),
                           cv::Scalar(0, 0, 0), true, false, CV_32F);

    // 拷进页锁定内存，为异步的 H2D 拷贝做准备
    memcpy(m_blob_pinned, m_blob.data, m_blob.total() * m_blob.elemSize());

    // 输入送进显存
    cudaMemcpyAsync(m_buffers[m_input_index], m_blob_pinned,
                    m_input_volume, cudaMemcpyHostToDevice, m_cuda_stream);

    // 执行推理（TRT 10 的 API，替代老的 enqueueV2）
    m_context->enqueueV3(m_cuda_stream);

    // 结果拷回页锁定内存
    cudaMemcpyAsync(m_rst, m_buffers[m_output_index],
                    m_output_volume, cudaMemcpyDeviceToHost, m_cuda_stream);

    // ★ 阻塞到整条流做完 —— 没有这一句就真的是异步了，但返回的指针也没法用。
    //   加上它之后，"async" 和 "sync" 在执行行为上没有任何区别。
    cudaStreamSynchronize(m_cuda_stream);

    return m_rst;
}

// 异步推理。
// ★★★ 本函数与上面的 syncInfer 【逐行完全相同】（唯一区别是函数名）★★★
//   也就是说这个后端根本没有实现异步流水线：同样在本函数返回前就
//   cudaStreamSynchronize 阻塞等待结果，返回的也是同一个成员缓冲区 m_rst。
//   本工程在 Detector 里写 infer_mode = "sync" 就是为了让代码读起来和实际
//   行为一致 —— 传 "async" 绝对【不会】带来任何性能提升，反而容易让人误以为
//   存在双缓冲/多请求并发。
//   真正有多缓冲实现的只有 OpenVINO 后端（见 network_deployment_openvino.cpp
//   的 asyncInfer/asyncInfer4，那边才有 2 路/4 路请求轮转）。
const float *TensorRTEngine::asyncInfer(const cv::Mat &pre_processed_image)
{
    cv::dnn::blobFromImage(pre_processed_image, m_blob, 1 / 255.0,
                           cv::Size(m_target_width, m_target_height),
                           cv::Scalar(0, 0, 0), true, false, CV_32F);

    memcpy(m_blob_pinned, m_blob.data, m_blob.total() * m_blob.elemSize());

    cudaMemcpyAsync(m_buffers[m_input_index], m_blob_pinned,
                    m_input_volume, cudaMemcpyHostToDevice, m_cuda_stream);

    m_context->enqueueV3(m_cuda_stream);

    cudaMemcpyAsync(m_rst, m_buffers[m_output_index],
                    m_output_volume, cudaMemcpyDeviceToHost, m_cuda_stream);

    cudaStreamSynchronize(m_cuda_stream);

    return m_rst;
}

// 释放顺序：流 → 页锁定内存 → 显存。
// ★ 先销毁流是为了保证流上排队的拷贝都结束了再动内存；
//   反过来先 free 显存再销毁流，理论上可能出现"流还在用已释放的地址"。
// ★ 这里没有显式 reset 三个 unique_ptr（runtime/engine/context），
//   它们由 unique_ptr 的默认删除器处理，析构顺序在成员声明顺序的逆序，
//   也就是 context → engine → runtime，刚好是 TRT 要求的依赖顺序。
TensorRTEngine::~TensorRTEngine()
{
    if (m_cuda_stream)
        cudaStreamDestroy(m_cuda_stream);
    if (m_blob_pinned)
        cudaFreeHost(m_blob_pinned);
    if (m_rst)
        cudaFreeHost(m_rst);
    if (m_buffers[m_input_index])
        cudaFree(m_buffers[m_input_index]);
    if (m_buffers[m_output_index])
        cudaFree(m_buffers[m_output_index]);
}

#endif

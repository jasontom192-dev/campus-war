#pragma once

// ============================================================================
// network_deployment_tensorrt.hpp —— ★ 本工程实际使用的推理后端
//
// 整个文件被 NNDEPLOYMENT_WITH_TENSORRT 包住：这个宏由 CMake 决定
// （本工程根 CMakeLists 里把 NNDEPLOYMENT_ENABLE_TENSORRT 设成 ON，
//   NNdeployment 自己的 CMakeLists 再把它翻成 -DNNDEPLOYMENT_WITH_TENSORRT=1）。
// ★ 注意这个开关是【二选一】而不是叠加：开了 TensorRT 就把 OpenVINO 关掉，
//   两个后端不会同时编进来。所以本工程里根本不存在 OpenVINOEngine 这个类。
//
// 数据流（本工程 640×480 输入）：
//   preProcessImage 给的 u8 BGR Mat
//     → cv::dnn::blobFromImage(1/255, swapRB=true, CV_32F)  ← BGR→RGB、归一化、NCHW、f32
//     → 页锁定内存 → cudaMemcpyAsync 到显存 → enqueueV3 → 拷回页锁定内存
//
// ★ 本后端的两个"名不副实"之处，本工程要知道：
//   1) syncInfer 和 asyncInfer 是【逐行相同】的，都带 cudaStreamSynchronize，
//      也就是说"异步"只是接口名字，实际是同步阻塞调用。Detector 里写 "sync"
//      只是为了名实相符，不是性能取舍。
//   2) async4 没有实现（基类默认实现会抛 logic_error "async4 infer not supported"）。
// ============================================================================

#ifndef NNDEPLOYMENT_WITH_TENSORRT
#define NNDEPLOYMENT_WITH_TENSORRT 0
#endif

#if NNDEPLOYMENT_WITH_TENSORRT

#include "network_inference.hpp"

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <NvInferRuntimeCommon.h>
#include <cuda_runtime_api.h>

#include <fstream>
#include <iostream>

// logger是TensorRT必要的类，创建运行时环境的报错信息只能通过logger传递
// ★ 这里把 kINFO 级别整个吞掉了，只留下 WARNING/ERROR 打到 stdout。
//   好处是启动时不会被 TRT 的一堆 INFO 刷屏；代价是"engine 和当前环境不匹配"
//   这类问题原本会以 INFO 形式提示，现在只能靠抛出来的 runtime_error 判断。
//   想排查反序列化失败，先把这个条件去掉。
class Logger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity != Severity::kINFO) // 屏蔽了所有的INFO等级信息，只显示警告和错误信息
        {
            std::cout << msg << std::endl;
        }
    }
};

class TensorRTEngine : public YOLOModel::InferenceEngine
{
public:
    // 构造 = 读 .engine 文件 → 反序列化 → 建 context → 从引擎里读输入输出形状
    // → cudaMalloc 显存 + cudaMallocHost 页锁定内存 → 建 CUDA 流。
    // ★ 全程没有 try/catch，失败就抛 std::runtime_error 出去，
    //   最终表现为"ArmorModel 构造失败"，本工程 Detector 启动即报错。
    // ★ 输入输出张量的名字是【硬编码】的 "images" / "output0"，
    //   换模型导出脚本时名字变了这里会直接报错（TRT 找不到张量）。
    TensorRTEngine(const YOLOModel::ModelConfig &model_config, const DebugConfig &debug_config);
    // 析构负责释放 CUDA 流、页锁定内存和显存。顺序是"先流后内存"，因为流上可能
    // 还有未完成的拷贝。
    ~TensorRTEngine();
    // 返回引擎输入张量的 W/H（本工程 640×480），给预处理用。
    // ★ 是从 .engine 里读出来的，不是代码里写死的 —— 换 engine 后预处理尺寸自动跟着变。
    int inputWidth() const override;
    int inputHeight() const override;
    // 同步推理。返回值指向【成员 m_rst】（页锁定内存），生命周期到下一次 infer 为止。
    const float *syncInfer(const cv::Mat &pre_processed_image) override;
    // ★ 和 syncInfer 逐行相同，同样是同步阻塞的（见文件头说明与 .cpp 里的注释）。
    const float *asyncInfer(const cv::Mat &pre_processed_image) override;

private:
    // TensorRT相关成员
    Logger m_logger;                                                  // 日志记录器
    std::unique_ptr<nvinfer1::IRuntime> m_runtime = nullptr;          // TensorRT运行时对象
    std::unique_ptr<nvinfer1::ICudaEngine> m_engine = nullptr;        // TensorRT推理引擎
    std::unique_ptr<nvinfer1::IExecutionContext> m_context = nullptr; // 执行上下文
    void *m_buffers[2] = {nullptr, nullptr};                          // GPU显存指针数组[输入, 输出]
    const int m_input_index = 0;                                      // 输入缓冲区索引
    const int m_output_index = 1;                                     // 输出缓冲区索引
    cudaStream_t m_cuda_stream = nullptr;                             // CUDA流用于异步操作

    // 数据处理成员
    cv::Mat m_blob;                // 预处理后的输入张量
    void *m_blob_pinned = nullptr; // 页锁定内存中的输入数据
    float *m_rst = nullptr;        // 页锁定内存中的原始推理结果
    // ★ m_rst 就是后处理拿到的那个 input_ptr 的来源。后处理只读不写，
    //   而且必须在【下一次 infer 之前】用完 —— 否则会被下一帧的输出覆盖。
    //   本工程是同步调用（infer 返回后立刻后处理完），所以不会撞上。
    int m_target_width = 0;        // 目标输入图像宽度
    int m_target_height = 0;       // 模型输入高度
    int m_output_anchors = 0;      // 输出张量的锚框数量
                                   // ★ 这个成员在本文件外没有任何使用者（上游遗留），
                                   //   本工程的 6300 实际走的是 m_infer_param.out_tensor_cols。
    size_t m_input_volume = 0;     // 输入数据量(NCHW形式)
                                   // ★ 单位是字节（初值是 sizeof(float) 再乘各维），
                                   //   不是元素个数 —— 名字容易看错。
    size_t m_output_volume = 0;    // 输出数据量(NHW)
                                   // 同上，单位是字节：21*6300*4 = 529200。
};

#endif

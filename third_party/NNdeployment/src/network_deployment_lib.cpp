#include "network_deployment_lib.hpp"
#include "network_inference.hpp"
#include "network_performance.hpp"
#include "network_postprocess.hpp"

#ifndef NNDEPLOYMENT_WITH_TENSORRT
#define NNDEPLOYMENT_WITH_TENSORRT 0
#endif

#ifndef NNDEPLOYMENT_WITH_OPENVINO
#define NNDEPLOYMENT_WITH_OPENVINO 1
#endif

#if NNDEPLOYMENT_WITH_OPENVINO
#include "network_deployment_openvino.hpp"
#endif

#if NNDEPLOYMENT_WITH_TENSORRT
#include "network_deployment_tensorrt.hpp"
#endif

#include <opencv2/core/persistence.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>

// 总模块：YOLOModel 构造、部署/后处理选择以及完整推理入口
//
// ============================================================================
// 本文件是部署库的装配车间，本工程最关心的三件事都在这里：
//
// 1) 选谁：构造函数按 deploy_way 建引擎（本工程 = TensorRTEngine），
//    再按 postprocess_mode 建后处理。auto_detect 时会【读引擎报出来的输出张量
//    形状】来反推模型是哪种，判断表如下（见下面构造函数的 switch）：
//        [25200,22] → V5InfantryPostProcessor   (v5 步兵)
//        [25,6300]  → V8InfantryPostProcessor   (v8 步兵，关键点带置信度)
//        [21,6300]  → V8_21InfantryPostProcessor ← ★ 本工程
//        [18,6300]  → RunePostProcessor         (打符)
//    都不匹配就抛 runtime_error，不会带着空后处理继续跑
//
// 2) 阈值：confidence_threshold 来自 config，但 NMS 阈值是 defaultNmsThreshold()
//    里【硬编码】的 0.2f —— 这是本工程文档里"nms_threshold 配置已失效"的出处
//
// 3) 每帧流程：netProcess 就是"预处理 → 推理 → 后处理"三段，
//    开启测速时在每段前后打点（steady_clock），不影响结果
// ============================================================================

// ==================== YOLOModel 实现 ====================

// namespace 中只包含字符串解析、枚举转换、配置读取和提示输出等局部工具，如需研究原理代码，无需着重研究此部分内容
namespace
{
using NetDeployWay = YOLOModel::NetDeployWay;
using NetInferMode = YOLOModel::NetInferMode;
using NetPostProcessMode = YOLOModel::NetPostProcessMode;
using ModelConfig = YOLOModel::ModelConfig;

// 将字符串统一转换为小写
std::string lowerString(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    return value;
}

using PerformanceClock = std::chrono::steady_clock;

// 计算两个时间点之间的微秒数
double elapsedMicroseconds(PerformanceClock::time_point start,
                           PerformanceClock::time_point stop)
{
    return std::chrono::duration<double, std::micro>(stop - start).count();
}

// 下面这几个 parse* 是字符串 → 枚举的唯一入口
// 特性：空串走默认值（infer_mode 默认 async、deploy_way 默认 openvino、
// postprocess_mode 默认 auto_detect），其余无法识别的值一律抛 runtime_error
// ★ 也就是说 "tensorrt "（多个空格）之类拼写错误不会静默降级到 OpenVINO，
//   而是当场炸掉 —— 这点对本工程是好事，配错了能在启动时发现
// 将字符串解析为推理模式
NetInferMode parseInferMode(const std::string &value)
{
    const std::string mode = lowerString(value);
    if (mode.empty() || mode == "async")
        return NetInferMode::async;
    if (mode == "sync")
        return NetInferMode::sync;
    if (mode == "async4")
        return NetInferMode::async4;
    throw std::runtime_error("不支持的infer_mode: " + value);
}

// 将字符串解析为部署后端
NetDeployWay parseDeployWay(const std::string &value)
{
    const std::string way = lowerString(value);
    if (way.empty() || way == "openvino")
        return NetDeployWay::openvino;
    if (way == "tensorrt")
        return NetDeployWay::tensorrt;
    throw std::runtime_error("不支持的deploy_way: " + value);
}

// 将字符串解析为后处理模式
NetPostProcessMode parsePostProcessMode(const std::string &value)
{
    const std::string mode = lowerString(value);
    if (mode.empty() || mode == "auto" || mode == "auto_detect")
        return NetPostProcessMode::auto_detect;
    if (mode == "v5infantry" || mode == "v5infantry_fourpoints")
        return NetPostProcessMode::v5infantry_fourpoints;
    if (mode == "v8infantry" || mode == "v8infantry_fourpoints")
        return NetPostProcessMode::v8infantry_fourpoints;
    if (mode == "v8infantry_21" || mode == "v8infantry_fourpoints_21")
        return NetPostProcessMode::v8infantry_fourpoints_21;
    if (mode == "lidar" || mode == "lidar_fourpoints")
        return NetPostProcessMode::lidar_fourpoints;
    if (mode == "rune" || mode == "rune_fivepoints")
        return NetPostProcessMode::rune_fivepoints;
    throw std::runtime_error("不支持的postprocess_mode: " + value);
}

// 将部署后端转换为可打印字符串
std::string toString(NetDeployWay way)
{
    switch (way)
    {
    case NetDeployWay::openvino:
        return "openvino";
    case NetDeployWay::tensorrt:
        return "tensorrt";
    default:
        return "error type";
    }
}

// 将推理模式转换为可打印字符串
std::string toString(NetInferMode way)
{
    switch (way)
    {
    case NetInferMode::sync:
        return "sync";
    case NetInferMode::async:
        return "async";
    case NetInferMode::async4:
        return "async4";
    default:
        return "error type";
    }
}

// 将后处理模式转换为可打印字符串
std::string toString(NetPostProcessMode way)
{
    switch (way)
    {
    case NetPostProcessMode::auto_detect:
        return "auto";
    case NetPostProcessMode::v5infantry_fourpoints:
        return "v5infantry";
    case NetPostProcessMode::v8infantry_fourpoints:
        return "v8infantry";
    case NetPostProcessMode::v8infantry_fourpoints_21:
        return "v8infantry_21";
    case NetPostProcessMode::lidar_fourpoints:
        return "lidar";
    case NetPostProcessMode::rune_fivepoints:
        return "rune";
    default:
        return "error type";
    }
}

// 打印当前模型配置及非标准组合提示
// ★ 这部分是【无条件打印】的（不受 DebugConfig 控制），每次构造模型都会往
//   stdout 吐几行，本工程启动时的 "兵种：v8infantry_21 推理模式：sync
//   部署方式：tensorrt" 就是它。看到这几行说明模型已经建好了
// ★ 提示文案是按"上游标准配置"写的：v8 步兵标准用 openvino+async，
//   本工程故意用 tensorrt+sync，所以这里会多打一行"注：..."，
//   那行提示对我们【没有意义】，不代表配错了
void printCurrentModelInfo(const ModelConfig &model_config)
{
    std::cout << "兵种：" << toString(model_config.postprocess_mode) << " "
              << "推理模式：" << toString(model_config.infer_mode) << " "
              << "部署方式：" << toString(model_config.deploy_way) << std::endl;

    switch (model_config.postprocess_mode)
    {
    case NetPostProcessMode::auto_detect:
        break;
    case NetPostProcessMode::v5infantry_fourpoints:
        if ((model_config.infer_mode != NetInferMode::async && model_config.infer_mode != NetInferMode::async4) ||
            model_config.deploy_way != NetDeployWay::openvino)
            std::cout << "注：标准情况下v5步兵应该使用" << toString(NetDeployWay::openvino) << "部署，" << toString(NetInferMode::async) << "推理" << std::endl;
        break;
    case NetPostProcessMode::v8infantry_fourpoints:
    case NetPostProcessMode::v8infantry_fourpoints_21:
        if ((model_config.infer_mode != NetInferMode::async && model_config.infer_mode != NetInferMode::async4) ||
            model_config.deploy_way != NetDeployWay::openvino)
            std::cout << "注：标准情况下v8步兵应该使用" << toString(NetDeployWay::openvino) << "部署，" << toString(NetInferMode::async) << "推理" << std::endl;
        break;
    case NetPostProcessMode::lidar_fourpoints:
        if (model_config.infer_mode != NetInferMode::sync || model_config.deploy_way != NetDeployWay::tensorrt)
            std::cout << "注：标准情况下雷达应该使用" << toString(NetDeployWay::tensorrt) << "部署，" << toString(NetInferMode::sync) << "推理" << std::endl;
        break;
    case NetPostProcessMode::rune_fivepoints:
        if (model_config.infer_mode != NetInferMode::sync || model_config.deploy_way != NetDeployWay::openvino)
            std::cout << "注：标准情况下打符应该使用" << toString(NetDeployWay::openvino) << "部署，" << toString(NetInferMode::sync) << "推理" << std::endl;
        break;
    }
    std::cout << std::endl;
}

// 从 JSON 的指定节点读取内部模型配置
//
// 读的字段（全部可选，缺了就用默认值，除了 xml 必须有）：
//   xml               模型文件名，和 JsonConfig.model_folder 拼成完整路径
//   infer_mode        → parseInferMode
//   deploy_way        → parseDeployWay
//   postprocess_mode  → parsePostProcessMode
//   device            缺省 "GPU"
//   score_threshold   置信度阈值，缺省 0.5；兼容旧配置里的 config_thresh
//
// ★★ 本工程要记住的坑：这里【没有】读 nms_threshold
//    所以 config.yaml 里写 nms_threshold 是完全无效的，NMS 阈值只能由
//    defaultNmsThreshold() 决定（armor 系列 = 0.2f）
// ★ 本工程没走这个入口（Detector 直接传参构造），列在这里只为对照
ModelConfig loadModelConfigFromJson(const JsonConfig &json_config)
{
    cv::FileStorage config(json_config.json_path, cv::FileStorage::READ);
    if (!config.isOpened())
        throw std::runtime_error("无法打开模型配置文件: " + json_config.json_path);

    const cv::FileNode node = config[json_config.model_key];
    if (node.empty())
        throw std::runtime_error("模型配置中不存在键: " + json_config.model_key);

    if (json_config.model_folder.empty())
        throw std::runtime_error("传入的模型文件夹路径为空: " + json_config.model_key);

    const std::string xml_name = static_cast<std::string>(node["xml"]);
    if (xml_name.empty())
        throw std::runtime_error("模型配置缺少xml文件名: " + json_config.model_key);

    const std::string model_path =
        (std::filesystem::path(json_config.model_folder) / xml_name).lexically_normal().string();

    const NetInferMode infer_mode = parseInferMode(static_cast<std::string>(node["infer_mode"]));
    const NetDeployWay deploy_way = parseDeployWay(static_cast<std::string>(node["deploy_way"]));
    const NetPostProcessMode postprocess_mode = parsePostProcessMode(static_cast<std::string>(node["postprocess_mode"]));

    std::string device = static_cast<std::string>(node["device"]);
    if (device.empty())
        device = "GPU";

    float confidence_threshold = 0.5f;
    if (!node["score_threshold"].empty())
        confidence_threshold = static_cast<float>(node["score_threshold"]);
    else if (!node["config_thresh"].empty())                // 兼容旧版本json配置文件
        confidence_threshold = static_cast<float>(node["config_thresh"]);

    return ModelConfig(model_path, infer_mode, deploy_way, device, confidence_threshold, postprocess_mode);
}
} // namespace

YOLOModel::InferenceEngine::InferenceEngine(const ModelConfig &model_config, const DebugConfig &debug_config) : m_model_config(model_config), m_debug_config(debug_config) {}

const float *YOLOModel::InferenceEngine::infer(const cv::Mat &pre_processed_image)
{
    switch (m_model_config.infer_mode)
    {
    case NetInferMode::sync:
        return syncInfer(pre_processed_image);
    case NetInferMode::async:
        return asyncInfer(pre_processed_image);
    case NetInferMode::async4:
        return asyncInfer4(pre_processed_image);
    }
    throw std::runtime_error("不支持的推理模式");
}

// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
// 【本工程最重要的硬编码之一】NMS 阈值的唯一来源
//
// 位置：src/network_deployment_lib.cpp，本函数，armor 分支返回 0.2f
// 值  ：0.2f（IOU 比例），适用于 v5 / v8 / v8_21 / lidar / auto_detect；
//       只有打符那一支是 100.f（那支的单位是像素距离，不是 IOU，别混）
//
// 这个值怎么传下去的：构造函数把它当第二个参数交给 PostProcessor 基类，
// 存进 m_nms_threshold，后处理再原样丢给 cv::dnn::NMSBoxes 的 nms_threshold
// 全库没有任何代码从配置文件读它 —— 工程文档里写的
// "NMS 阈值硬编码 0.2，config.yaml 里的 nms_threshold 已经不起作用"
// 说的就是这个函数（不是 config 读取处漏了字段，而是这里根本没给入口）
//
// 0.2 的含义：两个装甲板框 IOU 超过 0.2 就算重复，只保留分高的那个
// 属于相当激进的抑制（常规检测取 0.45~0.7）。对装甲板这种"同一块板可能被
// 相邻特征响应出多个框"的场景是有意为之；反过来说，想放宽就得改这里
// 注意它和 confidence_threshold（来自 config，本工程 0.5）是两回事
// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
float YOLOModel::defaultNmsThreshold(NetPostProcessMode mode)
{
    switch (mode)
    {
    case NetPostProcessMode::rune_fivepoints:
        return 100.f;   // 符模型 NMS 阈值为像素距离
    case NetPostProcessMode::v5infantry_fourpoints:
    case NetPostProcessMode::v8infantry_fourpoints:
    case NetPostProcessMode::v8infantry_fourpoints_21:
    case NetPostProcessMode::lidar_fourpoints:
    case NetPostProcessMode::auto_detect:
    default:
        // ★ 本工程（v8infantry_fourpoints_21）落到的就是这一行：0.2f
        return 0.2f;    // armor模型 NMS 阈值为 IOU 比例
    }
}

YOLOModel::YOLOModel(const std::string &model_path)
    : YOLOModel(model_path,
                "async",
                "openvino",
                "GPU",
                0.5f,
                "auto_detect",
                DebugConfig())
{}

YOLOModel::YOLOModel(const std::string &model_path,
                     const std::string &infer_mode,
                     const std::string &deploy_way,
                     const std::string &device,
                     float confidence_threshold,
                     const std::string &postprocess_mode,
                     const DebugConfig &debug_config)
    : YOLOModel(ModelConfig(model_path,
                            parseInferMode(infer_mode),
                            parseDeployWay(deploy_way),
                            device,
                            confidence_threshold,
                            parsePostProcessMode(postprocess_mode)),
                debug_config)
{}

YOLOModel::YOLOModel(const JsonConfig &json_config,
                     const DebugConfig &debug_config)
    : YOLOModel(loadModelConfigFromJson(json_config), debug_config)
{}

YOLOModel::~YOLOModel() noexcept = default;

// 真正的构造函数：其他所有重载最终都委托到这里
// 顺序很重要 —— 先建引擎（引擎在构造时就把 out_tensor_rows/cols 填好了），
// 后建后处理（auto_detect 要靠上面那两个值来判断用哪个后处理）
YOLOModel::YOLOModel(const ModelConfig &model_config, const DebugConfig &debug_config) : m_model_config(model_config), m_debug_config(debug_config)
{
    if (m_debug_config.calculate_speed_info)
        m_speed_stats = std::make_unique<MPT::SpeedStats>();

    // 根据部署方式选择openvino/tensorrt，目前仅雷达会使用tensorrt
    // ★ 本工程是 tensorrt 这一支（根 CMakeLists 里
    //   NNDEPLOYMENT_ENABLE_TENSORRT=ON，且该开关是"二选一"：
    //   开了 TRT 就把 OpenVINO 关掉，所以下面 openvino 分支根本没有编译进去，
    //   真传了 "openvino" 会在运行时抛 "OpenVINO support is disabled at build time"）
    // ★ 引擎构造失败（engine 文件不存在/不是本机 GPU 编译的）会在这里抛异常，
    //   传播出去就是 Detector 构造失败 —— 启动即报错，不会带病运行
    switch (m_model_config.deploy_way)
    {
    case NetDeployWay::openvino:
#if NNDEPLOYMENT_WITH_OPENVINO
        m_inference_engine = std::make_unique<OpenVINOEngine>(m_model_config, m_debug_config);
#else
        throw std::runtime_error("OpenVINO support is disabled at build time");
#endif
        break;
    case NetDeployWay::tensorrt:
#if NNDEPLOYMENT_WITH_TENSORRT
        m_inference_engine = std::make_unique<TensorRTEngine>(m_model_config, m_debug_config);
#else
        throw std::runtime_error("TensorRT support is disabled at build time");
#endif
        break;
    }

    // 根据后处理模式决定后处理模块，大部分情况下使用auto即可，除非模型结构改了
    //
    // auto_detect 的本质是"用输出张量形状反查模型身份"。这张表就是本工程的
    // 21×6300 契约的落点：本工程 Infantry-v8n 输出 [1,21,6300]，
    // 命中 rows==21 && cols==6300 → V8_21InfantryPostProcessor
    // （NMS 阈值也在这里被 defaultNmsThreshold() 一起塞进后处理，之后不可改。）
    // ★ 副作用：m_model_config.postprocess_mode 在这里被改写成具体模式，
    //   所以 supportsArmor()/打印信息看到的是 v8infantry_fourpoints_21 而不是 auto_detect
    // ★ 注意所有分支都没有 default，形状不匹配直接抛异常（好事：不静默出错）；
    //   但反过来说，如果将来有人加了新的后处理枚举值却忘了在下面加 case，
    //   m_postprocessor 会是空指针，netProcess 里解引用就会崩 —— 这是上游的隐患，本工程不改
    switch (m_model_config.postprocess_mode)
    {
    case NetPostProcessMode::auto_detect:
        {
            const int rows = m_inference_engine->m_infer_param.out_tensor_rows;
            const int cols = m_inference_engine->m_infer_param.out_tensor_cols;
            if (rows == 25200 && cols == 22)
            {
                m_model_config.postprocess_mode = NetPostProcessMode::v5infantry_fourpoints;
                m_postprocessor = std::make_unique<V5InfantryPostProcessor>(
                    m_model_config,
                    defaultNmsThreshold(m_model_config.postprocess_mode));
            }
            else if (rows == 25 && cols == 6300)
            {
                m_model_config.postprocess_mode = NetPostProcessMode::v8infantry_fourpoints;
                m_postprocessor = std::make_unique<V8InfantryPostProcessor>(
                    m_model_config,
                    defaultNmsThreshold(m_model_config.postprocess_mode));
            }
            else if (rows == 21 && cols == 6300)
            {
                // ★★★ 本工程走的就是这一个分支 ★★★
                // 输出 [1,21,6300]：0:4 颜色4类、4:13 兵种9类、13:21 四个关键点(x,y)
                // 解码细节见 V8_21InfantryPostProcessor::postProcessArmorMat
                m_model_config.postprocess_mode = NetPostProcessMode::v8infantry_fourpoints_21;
                m_postprocessor = std::make_unique<V8_21InfantryPostProcessor>(
                    m_model_config,
                    defaultNmsThreshold(m_model_config.postprocess_mode));
            }
            else if (rows == 18 && cols == 6300)
            {
                m_model_config.postprocess_mode = NetPostProcessMode::rune_fivepoints;
                m_postprocessor = std::make_unique<RunePostProcessor>(
                    m_model_config,
                    defaultNmsThreshold(m_model_config.postprocess_mode));
            }
            else
            {
                throw std::runtime_error("无法自动识别模型输出格式: [" + std::to_string(rows) + ", " + std::to_string(cols) + "]");
            }
        }
        break;
    case NetPostProcessMode::v5infantry_fourpoints:
        m_postprocessor = std::make_unique<V5InfantryPostProcessor>(
            m_model_config,
            defaultNmsThreshold(m_model_config.postprocess_mode));
        break;
    case NetPostProcessMode::v8infantry_fourpoints:
        m_postprocessor = std::make_unique<V8InfantryPostProcessor>(
            m_model_config,
            defaultNmsThreshold(m_model_config.postprocess_mode));
        break;
    case NetPostProcessMode::v8infantry_fourpoints_21:
        m_postprocessor = std::make_unique<V8_21InfantryPostProcessor>(
            m_model_config,
            defaultNmsThreshold(m_model_config.postprocess_mode));
        break;
    case NetPostProcessMode::lidar_fourpoints:
        m_postprocessor = std::make_unique<LidarPostProcessor>(
            m_model_config,
            defaultNmsThreshold(m_model_config.postprocess_mode));
        break;
    case NetPostProcessMode::rune_fivepoints:
        m_postprocessor = std::make_unique<RunePostProcessor>(
            m_model_config,
            defaultNmsThreshold(m_model_config.postprocess_mode));
        break;
    }

    // 调用namespace中的函数打印使用的模型信息，这部分不需要debug开启也打印
    printCurrentModelInfo(m_model_config);
}

//  完整的网络识别框架——主要流程：传入图片->预处理->推理->后处理->输出结果
//
// ==================== 本工程每帧走的就是这个函数 ====================
// 输入：BGR 的 cv::Mat（原始分辨率，不用自己 resize/转格式）+ my_color
// 输出：std::vector<NetArmorResult>，坐标已还原到【输入图】的像素坐标系
//
// 三段各自的落点（本工程配置 tensorrt+v8_21）：
//   预处理 m_inference_engine->preProcessImage   → src/preprocess/network_preprocess.cpp
//          resize 到 640×480（纯 resize，本例无 padding），结果仍是 u8 BGR 的 Mat；
//          同时把 origin_w/h、scale、pad 写进 m_infer_param
//   推理   m_inference_engine->infer              → src/inference/network_deployment_tensorrt.cpp
//          内部再走 blobFromImage 做 BGR→RGB、1/255 归一化、HWC→NCHW、转 f32，
//          然后 enqueueV3 + cudaStreamSynchronize，返回页锁定内存里的 f32 裸指针
//   后处理 m_postprocessor->postProcessArmorMat   → src/postprocess/network_postprocess.cpp
//          解码 → 颜色/兵种筛选 → 关键点还原到原图 → NMSBoxes → 组装 NetArmorResult
//
// 边界条件（都会返回空 vector，不抛异常）：
//   - 预处理失败（空图、图太狭长导致缩放后宽或高为 0）→ 提前 return {}
//   - 模型没检测到任何目标 / 所有候选被阈值和 NMS 滤掉 → 空 vector
// ★ 但"模型输出形状和后处理对不上"不算边界条件：后处理会往 stderr 打
//   "模型输出形状不匹配" 并返回空 vector。所以出现空结果且 stderr 有这行时，
//   是模型文件用错了，不是没检测到
std::vector<NetArmorResult> YOLOModel::netProcess(const cv::Mat &input_image, const int &my_color)
{
    // debug 的测速选项开启时进行各部分的测速
    const bool measure = m_speed_stats != nullptr;
    PerformanceClock::time_point phase_start{};
    double preprocess_us = 0.0;
    double inference_us = 0.0;
    double postprocess_us = 0.0;

    // 预处理
    if (measure)
        phase_start = PerformanceClock::now();
    cv::Mat pre_processed_mat = m_inference_engine->preProcessImage(input_image);
    if (measure)
        preprocess_us = elapsedMicroseconds(phase_start, PerformanceClock::now());
    if (pre_processed_mat.empty())
        return {};

    // 推理
    if (measure)
        phase_start = PerformanceClock::now();
    const float *infer_output = m_inference_engine->infer(pre_processed_mat);
    if (measure)
        inference_us = elapsedMicroseconds(phase_start, PerformanceClock::now());

    // 后处理
    if (measure)
        phase_start = PerformanceClock::now();
    std::vector<NetArmorResult> results = m_postprocessor->postProcessArmorMat(
        infer_output, m_inference_engine->m_infer_param, my_color);
    if (measure)
        postprocess_us = elapsedMicroseconds(phase_start, PerformanceClock::now());

    // 输出计时结果
    if (measure)
    {
        m_speed_stats->update(preprocess_us, inference_us, postprocess_us);
        m_speed_stats->printCurrentStats();
    }
    return results;
}

std::vector<NetRuneResult> YOLOModel::netProcess(const cv::Mat &input_image)
{
    const bool measure = m_speed_stats != nullptr;
    PerformanceClock::time_point phase_start{};
    double preprocess_us = 0.0;
    double inference_us = 0.0;
    double postprocess_us = 0.0;

    // 预处理
    if (measure)
        phase_start = PerformanceClock::now();
    cv::Mat pre_processed_mat = m_inference_engine->preProcessImage(input_image);
    if (measure)
        preprocess_us = elapsedMicroseconds(phase_start, PerformanceClock::now());
    if (pre_processed_mat.empty())
        return {};

    // 推理
    if (measure)
        phase_start = PerformanceClock::now();
    const float *infer_output = m_inference_engine->infer(pre_processed_mat);
    if (measure)
        inference_us = elapsedMicroseconds(phase_start, PerformanceClock::now());

    // 后处理
    if (measure)
        phase_start = PerformanceClock::now();
    std::vector<NetRuneResult> results = m_postprocessor->postProcessRuneMat(
        infer_output, m_inference_engine->m_infer_param);
    if (measure)
        postprocess_us = elapsedMicroseconds(phase_start, PerformanceClock::now());

    // 输出计时结果
    if (measure)
    {
        m_speed_stats->update(preprocess_us, inference_us, postprocess_us);
        m_speed_stats->printCurrentStats();
    }
    return results;
}

// 装甲板类后处理的集合。注意包不包含某个模式，是"这个模型出的是不是装甲板
// 结果"的唯一判据 —— interface 层就是靠它决定 ArmorModel 能不能构造
// ★ 这里读的 postprocess_mode 已经是 auto_detect 解析后的具体值，
//   所以本工程的 [21,6300] 模型能正确返回 true
// ★ auto_detect 本身不在列表里：如果哪天后处理没解析成功（理论上不会，
//   形状不匹配会先抛异常），supportsArmor() 会返回 false 并让构造失败
bool YOLOModel::supportsArmor() const
{
    return m_model_config.postprocess_mode == NetPostProcessMode::v5infantry_fourpoints ||
           m_model_config.postprocess_mode == NetPostProcessMode::v8infantry_fourpoints ||
           m_model_config.postprocess_mode == NetPostProcessMode::v8infantry_fourpoints_21 ||
           m_model_config.postprocess_mode == NetPostProcessMode::lidar_fourpoints;
}

// 神符后处理的集合。本工程不打符，恒为 false 的路径
bool YOLOModel::supportsRune() const
{
    return m_model_config.postprocess_mode == NetPostProcessMode::rune_fivepoints;
}

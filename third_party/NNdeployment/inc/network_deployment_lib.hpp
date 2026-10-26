#pragma once

// ============================================================================
// network_deployment_lib.hpp —— 部署库的中枢 YOLOModel
//
// 职责：把"部署方式(OpenVINO/TensorRT) × 推理模式(sync/async/async4) ×
// 后处理(5 种)"这三组配置收敛成一个对象，对外只暴露
// netProcess(img, my_color) 一个入口。真正做事的对象是它持有的两个成员：
//     m_inference_engine —— 预处理 + 推理（见 inc/inference/）
//     m_postprocessor    —— 解码 + NMS（见 inc/postprocess/）
//
// 完整的每帧调用链（本工程 TensorRT + V8_21 走的路径）：
//     ArmorModel::netProcess
//       └─ YOLOModel::netProcess                          ← 本文件声明
//            ├─ InferenceEngine::preProcessImage           src/preprocess/
//            ├─ InferenceEngine::infer → syncInfer         src/inference/network_deployment_tensorrt.cpp
//            └─ PostProcessor::postProcessArmorMat         src/postprocess/network_postprocess.cpp
//                 └─ V8_21InfantryPostProcessor::postProcessArmorMat
//
// ★ 本工程实际只走到的一条路：tensorrt + sync + V8_21InfantryPostProcessor
//   其余分支（OpenVINO 引擎、V5 后处理、雷达后处理、神符后处理）都是上游
//   为别的兵种/别的模型留的，本工程不启用，注释只写到"这里是什么"为止
// ============================================================================

#include "network_deployment_interface.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace MPT { struct SpeedStats; }

// 后端与后处理之间的参数：输出张量形状，以及预处理生成的原图坐标还原信息
//
// 这个结构是"推理引擎 → 后处理"的桥梁：引擎在构造时填 out_tensor_rows/cols，
// 在每帧 preProcessImage 时填 origin_width/height/scale/pad_x/pad_y，
// 后处理再拿它做两件事：
//   1) 判断输出形状对不对（各后处理开头的形状检查就是在比 rows/cols）；
//      也正因为这里有这个信息，auto_detect 才能反推该用哪个后处理
//   2) 把网络输出坐标还原回原图：原图 = (网络坐标 - pad) / scale
//
// 字段含义：
//   origin_width/height  原图尺寸（预处理前的 cv::Mat 的 cols/rows）
//   out_tensor_rows/cols 输出张量去掉 batch 维后的形状 [rows, cols]，
//                        例如本工程的 [21, 6300] → rows=21, cols=6300
//   scale                预处理时用的缩放系数 = min(目标宽/原宽, 目标高/原高)
//   pad_x/pad_y          纯 resize 无 padding 时为 0；走 padding 分支时是
//                        左右/上下的灰边宽度（见 network_preprocess.cpp）
// ★ scale/pad 是"预处理"和"后处理"之间唯一的约定，改预处理必须同步改后处理
// ★ 名字有点反直觉：scale 是"原图→模型"的系数，还原时是【除】它
struct InferParam
{
    int origin_width = 0;
    int origin_height = 0;
    int out_tensor_cols = 0;
    int out_tensor_rows = 0;
    float scale = 1.0;
    int pad_x = 0;
    int pad_y = 0;
};

// 各后端与后处理的实现类，只在这里前向声明；具体定义分散在 inc/inference 和
// inc/postprocess 下，YOLOModel 用 unique_ptr 持有它们，所以头文件不必互相包含
class OpenVINOEngine;
class TensorRTEngine;
class V5InfantryPostProcessor;
class V8InfantryPostProcessor;
class V8_21InfantryPostProcessor;
class LidarPostProcessor;
class RunePostProcessor;

// ==================== 主类定义 ====================
// 部署库真正的主类。构造时按 ModelConfig 建引擎 + 挑后处理，之后每帧调 netProcess
// ★ 构造开销大（读模型、反序列化、cudaMalloc），必须长期持有、不要每帧新建
// ★ 构造过程会抛异常（std::runtime_error / std::invalid_argument），
//   调用方要保证异常能被看到，而不是静默拿到一个坏对象
// ★ netProcess 不是线程安全的：引擎里有复用的 blob/输出缓冲，
//   后处理里还有跨帧的历史队列，一个实例只能被一个线程用
class YOLOModel
{
public:
    // 使用默认参数加载模型：async + openvino + GPU + auto_detect
    // 本工程不用（我们走 TensorRT）
    YOLOModel(const std::string &model_path);

    // 直接指定模型路径、推理模式和部署后端
    // 字符串参数在这里被 parseInferMode / parseDeployWay / parsePostProcessMode
    // 解析，解析不了会抛 runtime_error（不做静默回退）
    YOLOModel(const std::string &model_path,
              const std::string &infer_mode,
              const std::string &deploy_way,
              const std::string &device = "GPU",
              float confidence_threshold = 0.5f,
              const std::string &postprocess_mode = "auto_detect",
              const DebugConfig &debug_config = DebugConfig());
    // 从 JSON 的指定节点读取模型配置
    // ★ 只认 xml / infer_mode / deploy_way / postprocess_mode / device /
    //   score_threshold（旧名 config_thresh）这几个字段，nms_threshold 不读
    YOLOModel(const JsonConfig &json_config,
              const DebugConfig &debug_config = DebugConfig());
    YOLOModel() = delete;
    ~YOLOModel() noexcept;

    // 禁止拷贝（内部是引擎 + CUDA 资源，拷不动），允许移动
    // 移动之后源对象为空，所以接口层才需要 if (!m_model) 的兜底
    YOLOModel(const YOLOModel &) = delete;
    YOLOModel &operator=(const YOLOModel &) = delete;
    YOLOModel(YOLOModel &&) noexcept = default;
    YOLOModel &operator=(YOLOModel &&) noexcept = default;

    // 完整的一帧：预处理 → 推理 → 后处理 → 结果（坐标已还原到原图）
    // my_color 的含义见 interface 头文件；无结果时返回空 vector，
    // 【不会】用异常表达"没检测到"
    std::vector<NetArmorResult> netProcess(const cv::Mat &input_image, const int &my_color);
    std::vector<NetRuneResult> netProcess(const cv::Mat &input_image);

    // 判断当前模型是否使用装甲板后处理
    // ★ 判断依据是 m_model_config.postprocess_mode —— 注意这个字段在构造
    //   auto_detect 分支里被改写成了"解析出来的真实模式"，所以这里查的是
    //   解析后的结果，不是用户传进来的 "auto_detect"
    bool supportsArmor() const;
    // 判断当前模型是否使用神符后处理
    bool supportsRune() const;
    class InferenceEngine;
    class PostProcessor;

    // 部署后端。数字值没有序列化用途，只是枚举默认顺序
    enum class NetDeployWay
    {
        openvino = 0,
        tensorrt = 1      // 本工程用的
    };
    // 推理模式
    // ★ TensorRT 后端下 sync 和 async 走的是同一段代码（两个函数逐行相同），
    //   并不存在真正的异步流水线；async4 在 TRT 后端未实现，会抛异常
    //   OpenVINO 后端下三者才有实现上的区别（sync 单请求 / async 双缓冲 /
    //   async4 四缓冲，见 network_deployment_openvino.cpp）
    enum class NetInferMode
    {
        sync = 0,
        async = 1,
        async4 = 2
    };
    // 后处理模式 = "这套模型输出该怎么解码"
    // 和一个网络模型的绑定关系比想象中紧：改模型结构基本就得加一个枚举值
    // 本工程走 auto_detect → 被改写成 v8infantry_fourpoints_21
    enum class NetPostProcessMode
    {
        auto_detect = 0,             // 按输出张量形状自动挑，推荐值
        v5infantry_fourpoints = 1,   // 输出 [25200,22]，置信度是未 sigmoid 的 logit
        v8infantry_fourpoints = 2,   // 输出 [25,6300]，每关键点 3 个数（含关键点置信度）
        lidar_fourpoints = 3,        // 雷达模型 [26,2100]，类别直接编码了颜色
        rune_fivepoints = 4,         // 打符 [18,6300]，5 个关键点，NMS 用中心距
        v8infantry_fourpoints_21 = 5 // ★ 本工程：[21,6300]，每关键点只有 (x,y)
    };

    // 一份完整的模型配置。三个"直接传参 / 读 JSON / 走默认值"的入口最终都
    // 归到它上面，再由私有构造函数统一消费 —— 想加配置项时改这里就够
    // ★ 注意 postprocess_mode 会被构造过程改写：传 auto_detect 时最终会被
    //   替换成识别出来的具体模式（auto_detect 本身不是一种真正的后处理）
    struct ModelConfig
    {
        std::string model_path;
        NetPostProcessMode postprocess_mode;
        NetInferMode infer_mode;
        NetDeployWay deploy_way;
        std::string device;              // 只有 OpenVINO 后端会用到，TRT 后端忽略
        float confidence_threshold = 0.5f; // 后处理的候选置信度门限，不是 NMS 阈值

        ModelConfig() = delete;
        // 参数顺序和 YOLOModel 的四参构造重载【不一致】（这里 infer/deploy 在前、
        // device 在后，且 postprocess_mode 是最后一个），别按位置硬记
        ModelConfig(const std::string &path,
                    const NetInferMode &infer_mode = NetInferMode::async,
                    const NetDeployWay &deploy_way = NetDeployWay::openvino,
                    const std::string &device = "GPU",
                    float confidence = 0.5f,
                    const NetPostProcessMode &postprocess_mode = NetPostProcessMode::auto_detect)
            : model_path(path),
              postprocess_mode(postprocess_mode),
              infer_mode(infer_mode),
              deploy_way(deploy_way),
              device(device),
              confidence_threshold(confidence)
        {}
    };

private:
    // 所有公开构造函数最终都委托到这一个。真正的初始化都在 .cpp 里
    YOLOModel(const ModelConfig &model_config, const DebugConfig &debug_config = DebugConfig());

    // 返回该后处理模式使用的 NMS 阈值
    //
    // ★★ 本工程要记住的一件事：NMS 阈值是【硬编码】的，配置改不动它
    //    实现在 src/network_deployment_lib.cpp 的 YOLOModel::defaultNmsThreshold()，
    //    armor 系列（含本工程用的 v8infantry_fourpoints_21）返回 0.2f
    //    这个值是 IOU 比例，0.2 相当激进（两个框只要重叠超过 20% 就压掉一个）
    //    工程文档里说"config.yaml 的 nms_threshold 已失效"就是指这里
    //    —— 库没有任何读取该配置项的代码路径
    //    上游的注释说是"为兼容旧代码写死"，所以不要指望它可配
    static float defaultNmsThreshold(NetPostProcessMode mode);

    ModelConfig m_model_config;
    DebugConfig m_debug_config;
    // 非空即代表开启了测速（calculate_speed_info），netProcess 里用它当开关判断
    // 用 unique_ptr 而不是值成员，是为了不把 network_performance.hpp 拉进这个头文件
    std::unique_ptr<MPT::SpeedStats> m_speed_stats;

    // 二选一的引擎：TensorRT 或 OpenVINO，由 deploy_way 决定
    // 本工程是 TensorRTEngine
    std::unique_ptr<InferenceEngine> m_inference_engine;
    // 后处理：由 output 张量形状自动挑（或按 postprocess_mode 直接指定）
    // 本工程是 V8_21InfantryPostProcessor
    std::unique_ptr<PostProcessor> m_postprocessor;
};

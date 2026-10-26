#pragma once

// ============================================================================
// network_inference.hpp —— 推理引擎的抽象基类
//
// 两个具体实现：
//   TensorRTEngine  ← ★ 本工程用这个（network_deployment_tensorrt.hpp）
//   OpenVINOEngine  ← 本工程不编、不用（network_deployment_openvino.hpp）
//
// 基类只做两件事：
//   1) 定义"预处理 + 推理"的公共骨架：preProcessImage 是所有后端共用的一份
//      （实现在 src/preprocess/network_preprocess.cpp），
//      infer() 则按 infer_mode 把请求分发到 sync/async/async4 三个虚函数之一。
//   2) 持有 m_infer_param —— 它是预处理/推理与后处理之间唯一的契约，
//      引擎在构造和每帧预处理时往里填原图尺寸、缩放比、输出张量形状。
//
// ★ 三个推理虚函数不是每个后端都实现：
//    TensorRTEngine 只实现了 sync 和 async（而且两者是同一段代码），
//    async4 用基类默认实现 —— 直接抛 logic_error("async4 infer not supported")。
//    所以本工程传 infer_mode="async4" 会在第一帧炸掉，而不是静默降级。
// ★ 返回值 const float* 指向【引擎内部缓冲区】，只在下次 infer 前有效（见各后端说明）。
// ============================================================================

#include "network_deployment_lib.hpp"

#include <opencv2/core.hpp>

class YOLOModel::InferenceEngine
{
public:
    InferenceEngine(const ModelConfig &model_config, const DebugConfig &debug_config);
    virtual ~InferenceEngine() = default;

    // 模型输入尺寸。TensorRT 后端是从 .engine 里读出来的，
    // OpenVINO 后端是从模型输入形状读的；预处理的 resize 目标就是它们。
    virtual int inputWidth() const = 0;
    virtual int inputHeight() const = 0;
    // 返回供当前推理请求写入的预处理缓冲区。
    // ★ 默认返回空 Mat，表示"没有预分配缓冲，请自己新建一个"。
    //   TensorRT 后端没有 override（用这个默认实现）→ 预处理每帧新分配一次 Mat；
    //   OpenVINO 后端在 async/async4 下 override 成成员缓冲区
    //   （异步必须这样，否则请求还在读输入而 Mat 已经析构）。
    virtual cv::Mat acquirePreprocessBuffer() { return {}; }
    // 使用当前后端的输入尺寸、坐标参数和请求缓冲区预处理图像。
    // 非虚函数：所有后端共用一份实现（见 network_preprocess.cpp），
    // 差异通过 inputWidth()/inputHeight()/acquirePreprocessBuffer() 三个虚函数体现。
    cv::Mat preProcessImage(const cv::Mat &origin_image);
    // 按 infer_mode 分发到下面三个虚函数。纯转发，没有别的逻辑。
    const float *infer(const cv::Mat &pre_processed_image);
    virtual const float *syncInfer(const cv::Mat &pre_processed_image) = 0;
    virtual const float *asyncInfer(const cv::Mat &pre_processed_image) = 0;
    // 默认实现直接抛异常 —— 没实现 async4 的后端（TensorRT）会以异常方式拒绝，
    // 而不是返回一个空指针让上层崩在别处。
    virtual const float *asyncInfer4(const cv::Mat &pre_processed_image) { throw std::logic_error("async4 infer not supported"); }

    // ★ 预处理写、后处理读。字段含义见 network_deployment_lib.hpp 里的 InferParam。
    //   注意它属于引擎（每个引擎一份），所以 auto_detect 才能在构造后马上
    //   通过 m_inference_engine->m_infer_param 判断该建哪个后处理。
    InferParam m_infer_param;

protected:
    ModelConfig m_model_config;
    DebugConfig m_debug_config;
};

#pragma once

//推理引擎抽象基类：预处理、infer 分发、持有 InferParam
//TensorRT 只实现 sync/async（二者相同）；async4 默认抛异常

#include "network_deployment_lib.hpp"

#include <opencv2/core.hpp>

class YOLOModel::InferenceEngine
{
public:
    InferenceEngine(const ModelConfig &model_config, const DebugConfig &debug_config);
    virtual ~InferenceEngine() = default;

    //模型输入宽高；预处理 resize 到该尺寸
    virtual int inputWidth() const = 0;
    virtual int inputHeight() const = 0;
    //返回预处理缓冲；默认空，TensorRT 每帧新建
    virtual cv::Mat acquirePreprocessBuffer() { return {}; }
    //预处理：resize 并填 m_infer_param；各后端共用
    cv::Mat preProcessImage(const cv::Mat &origin_image);
    //按 infer_mode 分发到 sync/async/async4
    const float *infer(const cv::Mat &pre_processed_image);
    virtual const float *syncInfer(const cv::Mat &pre_processed_image) = 0;
    virtual const float *asyncInfer(const cv::Mat &pre_processed_image) = 0;
    //async4 默认抛异常；TensorRT 未实现
    virtual const float *asyncInfer4(const cv::Mat &pre_processed_image) { throw std::logic_error("async4 infer not supported"); }

    //预处理写、后处理读；auto_detect 靠它选后处理
    InferParam m_infer_param;

protected:
    ModelConfig m_model_config;
    DebugConfig m_debug_config;
};

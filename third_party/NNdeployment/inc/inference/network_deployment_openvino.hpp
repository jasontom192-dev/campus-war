#pragma once

// ============================================================================
// network_deployment_openvino.hpp —— OpenVINO 后端
//
// 【本工程不使用】tdt-learning4.4 走的是另一条分支：根 CMakeLists 里
// NNDEPLOYMENT_ENABLE_TENSORRT=ON，NNdeployment 的 CMakeLists 是二选一编译，
// 所以这个类【根本没有被编进本工程的库】，本工程也没有 OpenVINO 依赖
//
// 留在这里只是为了：1) 和 26_NNdeployment 上游保持一致；2) 将来要回到
// CPU/GPU 上的 OpenVINO 部署时有参考。它的主要特点是这个后端才有【真异步】：
// sync / async(2 路双缓冲) / async4(4 路流水) 三套请求池，
// 而 TensorRT 后端那边 sync 和 async 是同一段代码
// 另外 OpenVINO 后端是靠模型图的 ppp（PrePostProcessor）在内部做
// BGR→RGB / 归一化 / NHWC→NCHW 的，和 TRT 后端用 blobFromImage 的位置不同
// ============================================================================

#include "network_inference.hpp"

#include <openvino/openvino.hpp>

#include <array>
#include <cstddef>

class OpenVINOEngine : public YOLOModel::InferenceEngine
{
public:
    OpenVINOEngine(const YOLOModel::ModelConfig &model_config, const DebugConfig &debug_config);
    ~OpenVINOEngine() = default;

    int inputWidth() const override;
    int inputHeight() const override;
    // 根据推理模式返回对应 OpenVINO 请求的预处理缓冲区
    cv::Mat acquirePreprocessBuffer() override;
    const float *syncInfer(const cv::Mat &pre_processed_image) override;
    const float *asyncInfer(const cv::Mat &pre_processed_image) override;
    const float *asyncInfer4(const cv::Mat &pre_processed_image) override;

private:
    ov::CompiledModel m_compiled_model;
    ov::Shape m_input_shape;
    ov::Shape m_output_shape;
    ov::InferRequest m_sync_infer_request;
    std::array<ov::InferRequest, 2> m_async_infer_requests;
    std::array<cv::Mat, 2> m_async_preprocess_buffer;
    std::size_t m_async_submit_index = 0;
    std::size_t m_async_ready_index = 0;
    std::array<ov::InferRequest, 4> m_async4_infer_requests;
    std::array<cv::Mat, 4> m_async4_preprocess_buffer;
    std::size_t m_async4_submit_index = 0;
    std::size_t m_async4_ready_index = 0;

    void asyncStartup();
    void async4Startup();
};

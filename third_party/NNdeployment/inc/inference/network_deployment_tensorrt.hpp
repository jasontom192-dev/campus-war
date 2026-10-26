#pragma once

//TensorRT 推理后端：读 .engine、建 context、分配显存/页锁定内存/CUDA 流
//输入输出张量名固定为 images/output0；sync 与 async 实现相同

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

//TensorRT 日志：屏蔽 INFO，只输出 WARNING/ERROR
class Logger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity != Severity::kINFO) //屏蔽 INFO
        {
            std::cout << msg << std::endl;
        }
    }
};

class TensorRTEngine : public YOLOModel::InferenceEngine
{
public:
    //构造失败抛 runtime_error；张量名固定 images/output0
    TensorRTEngine(const YOLOModel::ModelConfig &model_config, const DebugConfig &debug_config);
    //释放 CUDA 流、页锁定内存和显存
    ~TensorRTEngine();
    //返回 engine 的输入宽高
    int inputWidth() const override;
    int inputHeight() const override;
    //同步推理，返回 m_rst（页锁定内存）
    const float *syncInfer(const cv::Mat &pre_processed_image) override;
    //与 syncInfer 相同，仍同步
    const float *asyncInfer(const cv::Mat &pre_processed_image) override;

private:
    //TensorRT 成员
    Logger m_logger;                                                  //日志
    std::unique_ptr<nvinfer1::IRuntime> m_runtime = nullptr;          //运行时
    std::unique_ptr<nvinfer1::ICudaEngine> m_engine = nullptr;        //引擎
    std::unique_ptr<nvinfer1::IExecutionContext> m_context = nullptr; //执行上下文
    void *m_buffers[2] = {nullptr, nullptr};                          //显存[输入,输出]
    const int m_input_index = 0;                                      //输入索引
    const int m_output_index = 1;                                     //输出索引
    cudaStream_t m_cuda_stream = nullptr;                             //CUDA 流

    //数据缓冲
    cv::Mat m_blob;                //输入 blob
    void *m_blob_pinned = nullptr; //页锁定输入
    float *m_rst = nullptr;        //页锁定输出
    //m_rst 是后处理拿到的输出缓冲，下次 infer 前有效
    int m_target_width = 0;        //输入宽
    int m_target_height = 0;       //输入高
    int m_output_anchors = 0;      //输出候选数（上游遗留，实际用 out_tensor_cols）
    size_t m_input_volume = 0;     //输入字节数
    size_t m_output_volume = 0;    //输出字节数，21*6300*4
};

#endif

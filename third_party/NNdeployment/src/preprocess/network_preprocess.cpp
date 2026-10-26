#include "network_inference.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>

// ============================================================================
// network_preprocess.cpp —— 预处理（所有后端共用这一份）
//
// 职责：把任意尺寸的原图变成"模型输入尺寸"的图，并把【坐标还原所需的信息】
// 写进 m_infer_param，供后处理把关键点从模型坐标系换算回原图坐标系
//
// ★ 这里只做【几何变换】，不做颜色和数值处理：
//   这一层输出的是 u8、BGR、仍是 HWC 的 cv::Mat；
//   BGR→RGB、÷255、HWC→NCHW、转 f32 是各推理引擎在 infer 时自己做的
//   （TensorRT 那边是 cv::dnn::blobFromImage(..., swapRB=true, 1/255, CV_32F)）
//   所以想让模型吃到正确数据，两处都得对 —— 排查"颜色反了/数值不对"时别只看这里
//
// ★ 本工程的实际情况：模型输入 640×480，比赛图 960×720、数据集 1440×1080
//   都是 4:3，长宽比和模型一致，所以走的是"纯 resize、无 padding"那条快路径，
//   pad_x = pad_y = 0，scale 就是简单的 640/原宽。关键点还原时
//   (v - 0) / scale 即可。另一条 padding 分支是给 V5（640×640）留的防御代码
// ============================================================================

// 由具体引擎实现的纯虚函数：返回模型输入宽/高
// TensorRT 后端是构造时从 engine 的输入张量读出来的（本工程 640×480）
// acquirePreprocessBuffer 是给异步推理用的"每请求一块缓冲"，
// TensorRT 没实现（用基类返回空 Mat），所以下面会每帧新分配一个 cv::Mat
cv::Mat YOLOModel::InferenceEngine::preProcessImage(const cv::Mat &origin_image)
{
    const int target_width = inputWidth();
    const int target_height = inputHeight();
    cv::Mat output_buffer = acquirePreprocessBuffer();

    // 先把原图尺寸记下来 —— 后处理靠它做越界检查（关键点必须在原图范围内），
    // 也靠它算还原后的坐标范围。★ 注意这几行写在下面的空图检查【之前】，
    //   所以空图时 origin_* 会是 0，但那种情况函数直接 return {}，后处理不会跑
    m_infer_param.origin_height = origin_image.rows;
    m_infer_param.origin_width = origin_image.cols;
    m_infer_param.pad_x = 0;
    m_infer_param.pad_y = 0;

    if (origin_image.empty() || target_width <= 0 || target_height <= 0)
        return {};

    // scale = 目标尺寸 / 原图尺寸，取【小的那个比例】——
    // 这样缩完两边都不会超过目标尺寸，另一维留白。等比缩放，不拉伸
    // ★ 这个值同时是后处理还原坐标时唯一的除数字，两处必须一致
    m_infer_param.scale = std::min(static_cast<float>(target_height) / origin_image.rows,
                                   static_cast<float>(target_width) / origin_image.cols);
    // 注意static_cast是截断，round确保不会因小数误差导致错误截断
    const int new_width = std::min(
        target_width,
        static_cast<int>(std::round(origin_image.cols * m_infer_param.scale)));
    const int new_height = std::min(
        target_height,
        static_cast<int>(std::round(origin_image.rows * m_infer_param.scale)));

    if (new_width == 0 || new_height == 0)
    {
        std::cout << "原图过于狭长,缩放导致宽度或高度为0" << std::endl;
        return {};
    }

    if (output_buffer.empty())
        output_buffer = cv::Mat(target_height, target_width, CV_8UC3);

    // 正常情况下需要确保模型输入的长宽比等于原图长宽比，避免padding。26赛季中所有图片尺寸均为1440*1080，对应的模型输入尺寸为640*480
    //
    // ★★★ 本工程（640×480 输入 + 4:3 的图）走的就是这一条快路径：
    //   等比缩放后刚好填满整个输入，不做 padding、不做 letterbox，
    //   一个 cv::resize 直接出结果。因为 pad 是 0，后处理里的坐标还原
    //   简化成 (v - 0) / scale —— 这也是"关键点会精确还原回原图坐标"的原因
    // ★ 这里用的是 INTER_LINEAR（双线性），和训练/导出时的 resize 方式要对得上，
    //   换成 INTER_AREA 之类会带来细微的坐标偏移（关键点是回归出来的坐标，
    //   对输入分布敏感）
    if (new_height == target_height && new_width == target_width)
    {
        cv::resize(origin_image, output_buffer, cv::Size(target_width, target_height), 0, 0, cv::INTER_LINEAR);
        return output_buffer;
    }

    // 长宽比不同情况下，需要copy一次实现padding，此处为防御代码，正常来说不应该进入（V5模型训练时仍然是 640 * 640，则此处会进行padding）
    // ★ 本工程进不到这里（不走 V5）。列在这里是为了说明 pad_x/pad_y 什么时候非 0：
    //   一旦走这条路，后处理必须做 (v - pad) / scale，否则关键点会整体平移
    // ★ padding 填的是 (124,124,124) 灰 —— 这是 YOLO 系列的惯例灰度值
    //   （不是 0，也不是 OpenVINO 那套异步预热用的 128），别随手改。这里填的
    //   是 BGR 三通道同一个值，所以顺序无所谓
    cv::Mat resized_view;
    cv::resize(origin_image, resized_view, cv::Size(new_width, new_height), 0, 0, cv::INTER_LINEAR);

    m_infer_param.pad_x = std::max(0, (target_width - new_width) / 2);
    m_infer_param.pad_y = std::max(0, (target_height - new_height) / 2);
    cv::copyMakeBorder(resized_view,
                       output_buffer,
                       m_infer_param.pad_y,
                       target_height - new_height - m_infer_param.pad_y,
                       m_infer_param.pad_x,
                       target_width - new_width - m_infer_param.pad_x,
                       cv::BORDER_CONSTANT,
                       cv::Scalar(124, 124, 124));
    return output_buffer;
}

#pragma once

// ============================================================================
// 【移植替换件 · 2026-10-04】ImgViz 空实现
//
// 原版把调试图像丢进 GUI 线程按窗口名显示（OpenCV highgui）。移植进
// tdt-learning4.5 后：离线工具自己画图、ROS 节点自己 cv::imshow，这里只保留
// 同名 API（power_rune 只用到 enqueue_image_zero_copy），全部空实现
// ============================================================================

#include <opencv2/opencv.hpp>

#include <string_view>

class ImgViz
{
public:
    static void init(bool /*enabled*/) {}
    static bool enabled() { return false; }

    static void enqueue_image_copy(std::string_view /*window_name*/, const cv::Mat & /*image*/) {}
    static void enqueue_image_zero_copy(std::string_view /*window_name*/, const cv::Mat & /*image*/) {}
};

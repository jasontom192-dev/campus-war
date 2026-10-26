#pragma once

// ============================================================================
// network_postprocess.hpp —— 后处理族的基类与各兵种实现声明
//
// 一个后处理 = "一种输出张量布局的解码器 + 该模型专属的过滤规则"
// 所有实现共享同一份约定：
//   输入：引擎吐出来的 f32 裸指针（形状靠 InferParam 描述，不是 cv::Mat）
//        + InferParam（原图尺寸/缩放/边框，用来把关键点还原回原图）
//   输出：NetArmorResult / NetRuneResult 数组，坐标已经是【原图像素坐标】
//
// 本工程只实例化 V8_21InfantryPostProcessor（本文件第 54 行那个类），
// 其余四个（v5 / v8 / lidar / rune）是上游给别的兵种和打符用的，本工程不构造
//
// ★ 裸指针约定：input_ptr 指向的是【推理引擎内部】的缓冲区
//   （TensorRT 后端是页锁定内存 m_rst，OpenVINO 后端是 output tensor 的数据），
//   生命周期只保证到下一次 infer 调用之前。后处理必须在同一帧内用完，
//   本工程的后处理是同步跑完就返回拷贝（结果里都是值），所以没有悬垂问题
// ★ 后处理【有状态】：装甲板那几个实现里带 m_last_results（最近 5 帧历史），
//   用来滤"被打中变白"的装甲板。所以同一个 PostProcessor 只能被一个线程顺序调用
// ============================================================================

#include "network_deployment_lib.hpp"

#include <opencv2/core.hpp>

#include <queue>
#include <string>
#include <vector>

class YOLOModel::PostProcessor
{
public:
    // model_config 提供 confidence_threshold；nms_threshold 由
    // YOLOModel::defaultNmsThreshold() 硬编码传入（本工程 armor = 0.2f）
    PostProcessor(const ModelConfig &model_config, float nms_threshold);
    virtual ~PostProcessor() = default;

    // 基类默认实现是抛异常（"这个后处理不支持装甲板结果"），
    // 各子类按自己能不能出装甲板结果来 override
    // ★ 返回值的产生方式：只在内部 vector 里构造，最后 emplace_back 搬出去，
    //   不会返回指向引擎缓冲区的引用
    virtual std::vector<NetArmorResult> postProcessArmorMat(const float *input_ptr, const InferParam &infer_param, const int &my_color) { throw std::logic_error("detect postprocess not supported"); };

    virtual std::vector<NetRuneResult> postProcessRuneMat(const float *input_ptr, const InferParam &infer_param) { throw std::logic_error("detect postprocess not supported"); };

protected:
    ModelConfig m_model_config;
    // 由 defaultNmsThreshold() 决定，构造后不再改变 —— 没有 setter，
    // 也不读配置文件。想调 NMS 只能改 defaultNmsThreshold() 的返回值
    float m_nms_threshold = 0.f;
};

class V5InfantryPostProcessor : public YOLOModel::PostProcessor
{
public:
    V5InfantryPostProcessor(const YOLOModel::ModelConfig &model_config, float nms_threshold);
    ~V5InfantryPostProcessor() = default;

    std::vector<NetArmorResult> postProcessArmorMat(const float *input_ptr, const InferParam &infer_param, const int &my_color) override;

private:
    std::vector<std::string> m_armor_names = {"哨兵", "英雄", "工程", "3号步兵", "4号步兵", "5号步兵", "前哨站", "基地（底部）", "基地（顶部）"};
    std::vector<std::string> m_color_names = {"蓝色", "红色", "白色", "紫色"};
    std::queue<std::vector<NetArmorResult>> m_last_results;
};

class V8InfantryPostProcessor : public YOLOModel::PostProcessor
{
public:
    V8InfantryPostProcessor(const YOLOModel::ModelConfig &model_config, float nms_threshold);
    ~V8InfantryPostProcessor() = default;

    std::vector<NetArmorResult> postProcessArmorMat(const float *input_ptr, const InferParam &infer_param, const int &my_color) override;

private:
    std::vector<std::string> m_armor_names = {"哨兵", "英雄", "工程", "3号步兵", "4号步兵", "5号步兵", "前哨站", "基地（底部）", "基地（顶部）"};
    std::vector<std::string> m_color_names = {"蓝色", "红色", "白色", "紫色"};
    std::queue<std::vector<NetArmorResult>> m_last_results;
};

// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
// 【本工程用的就是这一个】V8 步兵 21 维输出后处理
//
// 对应模型：Infantry-v8n-fp16-20260726-D1.8w-B16（本工程 models/infantry_v8n.engine）
// 输入张量：images [1, 3, 480, 640] f32，预处理后为 RGB / 0~1 / NCHW
// 输出张量：output0 [1, 21, 6300] f32，布局（第 0 维是 batch，下面说的都是那 21 行）：
//
//     [0 : 4]   颜色 4 类，argmax → color_id  0=蓝 1=红 2=白 3=紫
//     [4 : 13]  兵种 9 类，argmax → armor_id  0=哨兵 1=英雄 2=工程 3=3号步兵
//                                             4=4号步兵 5=5号步兵 6=前哨站
//                                             7=基地(底部) 8=基地(顶部)
//     [13: 21]  4 个关键点，每个点只有 (x, y) 两个数，共 8 个
//               顺序固定为：左上 → 左下 → 右下 → 右上（训练时就定死的）
//
// 6300 是候选数（anchor/query 数），所以内存排布是【列优先视角】：
// 第 f 个特征、第 c 个候选位于 data[f * 6300 + c]
// 这一点和 v5/v8-25 那两套（每候选连续 22/25 个数）刚好相反，
// 是本文件里最容易读反的地方 —— 见 postProcessArmorMat 里 CandidateView 的
// candidate_stride=1 / feature_stride=6300
//
// ★ 和 V8InfantryPostProcessor（25 维）的唯一区别：这里是 21 维，
//   每个关键点只有 xy、没有关键点置信度；25 维那套每点占 3 个数
//   两者 6300 一样、类别/颜色区段一样，但关键点区的偏移和步长不同：
//   本文件是 13 + i*2，25 维那套是 13 + i*3
//
// ★ 本工程为什么选它：网络是端到端的，颜色和兵种直接由网络给出，
//   所以工程里原来的 tiny_resnet 数字分类器整条链路都删掉了
// ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
class V8_21InfantryPostProcessor : public YOLOModel::PostProcessor
{
public:
    V8_21InfantryPostProcessor(const YOLOModel::ModelConfig &model_config, float nms_threshold);
    ~V8_21InfantryPostProcessor() = default;

    std::vector<NetArmorResult> postProcessArmorMat(const float *input_ptr, const InferParam &infer_param, const int &my_color) override;

private:
    // 兵种名的下标就是 armor_id —— 本工程 Detector 的 kArmorNames[9]
    // 必须和这里的顺序逐个对齐（顺序是模型训练时定的，不是随便排的）
    std::vector<std::string> m_armor_names = {"哨兵", "英雄", "工程", "3号步兵", "4号步兵", "5号步兵", "前哨站", "基地（底部）", "基地（顶部）"};
    // 颜色名的下标就是 color_id：0 蓝 1 红 2 白 3 紫
    std::vector<std::string> m_color_names = {"蓝色", "红色", "白色", "紫色"};
    // 最近 5 帧的装甲板结果，只给"白色装甲板要不要保留"这个判断用
    // （见 .cpp 里的 shouldAppendArmorResult）。构造时被塞进 5 个空 vector，
    // 每帧 pop 一个 push 一个，等于一个容量 5 的滑动窗口
    // ★ 这就是后处理不可重入的原因：它是跨帧状态，不是每帧重建的
    std::queue<std::vector<NetArmorResult>> m_last_results;
};

class LidarPostProcessor : public YOLOModel::PostProcessor
{
public:
    LidarPostProcessor(const YOLOModel::ModelConfig &model_config, float nms_threshold);
    ~LidarPostProcessor() = default;

    std::vector<NetArmorResult> postProcessArmorMat(const float *input_ptr, const InferParam &infer_param, const int &my_color) override;

private:
    std::vector<std::string> m_armor_names = {"蓝色1号英雄", "蓝色2号工程", "蓝色3号步兵", "蓝色4号步兵", "蓝色哨兵",
                                              "红色1号英雄", "红色2号工程", "红色3号步兵", "红色4号步兵", "红色哨兵"};
    std::vector<std::string> m_color_names = {"蓝色", "红色"};
};

class RunePostProcessor : public YOLOModel::PostProcessor
{
public:
    RunePostProcessor(const YOLOModel::ModelConfig &model_config, float nms_threshold);
    ~RunePostProcessor() = default;

    std::vector<NetRuneResult> postProcessRuneMat(const float *input_ptr, const InferParam &infer_param) override;

private:
    std::vector<std::string> m_class_names = {"未激活", "小符已激活", "大符已激活"};
};

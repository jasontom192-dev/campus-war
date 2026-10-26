#include "network_postprocess.hpp"

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstddef>
#include <iostream>
#include <numeric>
#include <queue>
#include <sstream>
#include <utility>

// ==================== 后处理模块 ====================
// 后处理模块：v5 armor、v8 armor、lidar 和 rune 四类结果解析
//
// ============================================================================
// 本工程（tdt-learning4.4）只用到本文件里的【V8_21InfantryPostProcessor】，
// 也就是下面这一节："v8 21维步兵实现"，函数
// V8_21InfantryPostProcessor::postProcessArmorMat
// 其余三节（V5 / V8-25 / 雷达 / 打符）是上游给别的兵种和打符节点准备的，
// 本工程既不会构造也不会执行，只做简短说明
//
// 所有装甲板后处理的共同骨架（看懂一个就看懂全部）：
//   1. 形状校验：用 InferParam 的 rows/cols 确认模型和本后处理是配套的，
//      不匹配就打 stderr 并返回空 vector —— 【不会抛异常】
//   2. 遍历候选（v5 是按行 25200，v8 系列是按列 6300），
//      用 CandidateView 把"第几个候选、第几个特征"换算成裸指针下标
//   3. 三路解码：颜色 argmax → color_id，兵种 argmax → armor_id，
//      关键点 (x,y) → 按 (v - pad) / scale 还原到原图坐标
//   4. 过滤：紫色必丢；和我方同色的丢（my_color）；白板走历史判定；
//      置信度低于 confidence_threshold 的丢
//   5. 几何过滤 + NMS：cv::dnn::NMSBoxes，阈值 = m_nms_threshold
//      （硬编码来的 0.2，见 network_deployment_lib.cpp 的 defaultNmsThreshold）
//   6. 组装 NetArmorResult：points 顺序保持"左上 左下 右下 右上"不变，
//      这一步【不做任何重排】；重排是本工程 Detector 自己的事
//
// ★ 关于置信度的两种世界（容易看混）：
//   V5 的输出是未过 sigmoid 的 logit，所以它先把阈值取 logit（反 sigmoid）再比；
//   V8/V8_21/打符 的输出【假定 sigmoid 已经烘进模型图里】，0~1 直接用
//   本工程的 Infantry-v8n 属于后者，score_threshold=0.5 是直接比 0~1 的值
// ============================================================================

// ==================== 部署流程辅助区域 ====================
namespace
{
// 将每个候选项封装，便于直接进行张量计算
//
// ★ 这是本文件里最需要先看懂的一个东西，因为它决定了"输出张量到底怎么读"
// 输出张量在内存里是一维的，但我们要按 [特征][候选] 或 [候选][特征] 两种
// 视角去访问，靠的就是这两个 stride：
//
//   V5（[25200,22]，每行一个候选）：
//       candidate_stride = 22, feature_stride = 1
//       → 候选 i 的特征 f 在 data[i*22 + f]
//   V8_21（[21,6300]，每列一个候选）：
//       candidate_stride = 1, feature_stride = 6300
//       → 候选 c 的特征 f 在 data[f*6300 + c]
//
// 这两组参数是构造 CandidateView 时按后处理各自的布局传进来的，
// 传反了不会报错，只会读到一堆乱七八糟的数（表现为检出全乱或全空）——
// 改后处理时这里是第一嫌疑点
struct CandidateView
{
    const float *data = nullptr;
    std::size_t candidate_index = 0;
    std::size_t candidate_stride = 0;
    std::size_t feature_stride = 0;

    // 按特征索引读取当前候选项的数据
    float operator[](std::size_t feature_index) const noexcept
    {
        return data[candidate_index * candidate_stride + feature_index * feature_stride];
    }
};

// 计算装甲板四个关键点的最小外接矩形
cv::Rect2d getArmorBoundingRect(const NetArmorResult &result)
{
    double min_x = result.points[0].x;
    double max_x = result.points[0].x;
    double min_y = result.points[0].y;
    double max_y = result.points[0].y;
    for (int i = 1; i < 4; ++i)
    {
        min_x = std::min(min_x, result.points[i].x);
        max_x = std::max(max_x, result.points[i].x);
        min_y = std::min(min_y, result.points[i].y);
        max_y = std::max(max_y, result.points[i].y);
    }
    return cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
}

// 计算两个装甲板外接矩形的交并比
double getArmorIoU(const NetArmorResult &lhs, const NetArmorResult &rhs)
{
    const cv::Rect2d lhs_rect = getArmorBoundingRect(lhs);
    const cv::Rect2d rhs_rect = getArmorBoundingRect(rhs);
    const cv::Rect2d intersection = lhs_rect & rhs_rect;
    const double intersection_area = intersection.area();
    if (intersection_area <= 0.0)
        return 0.0;

    const double union_area = lhs_rect.area() + rhs_rect.area() - intersection_area;
    if (union_area <= 0.0)
        return 0.0;

    return intersection_area / union_area;
}

} // namespace

// 装甲板后处理流程

// 构造白色装甲板判定使用的初始历史队列
// 塞 5 个空 vector 进去，等于"前 5 帧什么都没有"。之后每帧 pop 一个 push 一个，
// 队列长度恒为 5 —— 想改历史长度（比如 3 帧）要同时改这里的 push 次数
std::queue<std::vector<NetArmorResult>> buildInitialArmorHistory()
{
    std::queue<std::vector<NetArmorResult>> last_results;
    last_results.push({});
    last_results.push({});
    last_results.push({});
    last_results.push({});
    last_results.push({});
    return last_results;
}

// 根据颜色和历史结果判断当前白色装甲板结果是否应保留
//
// 这是 my_color 三处过滤里"最绕的那一处"，也是全库唯一跨帧的逻辑：
//   - 非白板（color_id != 2）直接放行 —— 红蓝板永远保留，不看历史
//   - my_color == 2（本工程用的值）时白板也全部放行，等于不做这套判断
//   - 否则（my_color 0/1，真正的比赛模式）白板要"自证身份"：
//     最近 5 帧里必须出现过【同兵种 + 位置重合(IOU > 0.3) + 我方颜色】的板，
//     才认为这块白板是"我方刚打中、正在变白的敌方板"而保留；
//     找不到就先丢掉。目的是把"装甲板未激活/无敌时显示成白色"这种干扰滤掉
//
// ★ 0.3 是这里第二个硬编码阈值（第一个是 NMS 的 0.2），同样不可配
// ★ 本工程因为固定传 my_color=2，这个函数进不到历史分支，等于恒返回 true；
//   白/紫的过滤挪到了 Detector 里按 color_id 做
// ★ 参数 last_results 是【按值传】的（函数内部还 pop 了它），
//   每调一次就拷贝一次队列里的 5 组结果 —— 这是上游的一个性能小坑，
//   对功能没影响，本工程不改（保持和上游一致）
bool shouldAppendArmorResult(const NetArmorResult &result, const int my_color, std::queue<std::vector<NetArmorResult>> last_results)
{
    // 仅白色装甲板额外判断。my_color == 2 表示测试模式，全部识别
    if ((result.color_id != 2) || (my_color ==2))
        return true;

    while (!last_results.empty())
    {
        const std::vector<NetArmorResult> &history_results = last_results.front();
        for (const NetArmorResult &history_result : history_results)
        {
            // 历史装甲板为待识别颜色，类别相同且 IOU 通过，可提供自瞄识别
            if (((my_color == 0 && history_result.color_id == 0) ||
                 (my_color == 1 && history_result.color_id == 1)) &&
                history_result.armor_id == result.armor_id &&
                getArmorIoU(history_result, result) > 0.3) 
                return true;
        }
        last_results.pop();
    }

    return false;
}

// 保存后处理所需的模型配置和非极大值抑制阈值
//
// 基类构造，被所有子类调用。两个成员：
//   m_model_config   只用到里面的 confidence_threshold（本工程来自 config.yaml
//                    的 score_threshold），后处理不关心部署方式和设备
//   m_nms_threshold  NMS 的 IOU 门限，由 YOLOModel::defaultNmsThreshold() 硬编码
//                    决定（armor 系列 = 0.2f）。构造后再无任何修改入口，
//                    配置文件也改不动它 —— 这就是"nms_threshold 已失效"的机制
YOLOModel::PostProcessor::PostProcessor(const ModelConfig &model_config, float nms_threshold)
    : m_model_config(model_config), m_nms_threshold(nms_threshold) {}

// ==================== v5步兵实现 ====================
// 【本工程不使用】对应输出 [25200, 22]（每行一个候选）。只在旧 v5 步兵模型上跑
// 和 V8 系最大的两点不同，换模型时容易搞混，记一下：
//   1) 每行一个候选，CandidateView 的 stride 是 (22, 1)，不是 (1, 6300)
//   2) 输出没做 sigmoid：它先把 confidence_threshold 取反 sigmoid
//      （raw_confidence_threshold = log(t/(1-t))）再和原始 logit 比，
//      等价于"先 sigmoid 再按 0~1 阈值筛"，但没有真的对每个候选算指数，
//      是省时间的写法。V8/V8_21 反过来，假定 sigmoid 已经在图里
// 初始化 V5 步兵后处理器及装甲板历史队列
V5InfantryPostProcessor::V5InfantryPostProcessor(const YOLOModel::ModelConfig &model_config, float nms_threshold)
    : PostProcessor(model_config, nms_threshold), m_last_results(buildInitialArmorHistory()) {}

// 解析 V5 步兵模型输出并生成装甲板检测结果
std::vector<NetArmorResult> V5InfantryPostProcessor::postProcessArmorMat(const float *input_ptr, const InferParam &infer_param, const int &my_color)
{
    const float confidence_threshold = m_model_config.confidence_threshold;
    const float nms_threshold = m_nms_threshold;

    if (!input_ptr || infer_param.out_tensor_cols != (4 * 2 + 1 + 4 + 9))
    {
        std::cerr << "模型输出形状不匹配，请检查模型路径" << std::endl;
        return {};
    }

    std::vector<int> class_ids_temp;         // 装甲板类别ID
    std::vector<float> confidences_temp;     // 检测置信度（用于NMS）——注意：为了对接自瞄代码，score是double，但是nms要求confidence必须是float，所以这里用的是float，到最后强转为double
    std::vector<int> sizes_temp;             // 装甲板尺寸标志（0=小装甲板，1=大装甲板）
    std::vector<int> colors_temp;            // 装甲板颜色类别（0=蓝，1=红，2=白，3=紫）
    std::vector<cv::Rect> boxes_temp;        // 边界框（用于OpenCV NMS算法）
    std::vector<cv::Point2d> keypoints_temp; // 装甲板四个关键点坐标
    std::vector<int> indices_temp;           // NMS后保留的检测框索引结果
    std::vector<NetArmorResult> results;     // 完整的装甲板检测结果

    // v5的置信度没做sigmoid，此处对confidence阈值做反sigmoid，等价于对所有置信度做了sigmoid之后按0-1阈值筛选
    const double raw_confidence_threshold = std::log(static_cast<double>(confidence_threshold) / (1.0 - static_cast<double>(confidence_threshold)));
    const float score_threshold = confidence_threshold;

    // V5 输出为 25200*22，每行是一组候选结果
    for (int candidate_index = 0; candidate_index < infer_param.out_tensor_rows; ++candidate_index)
    {
        // 封装候选项，便于后续直接当作张量计算
        const CandidateView candidate{input_ptr, static_cast<std::size_t>(candidate_index),
                                      static_cast<std::size_t>(infer_param.out_tensor_cols), 1};

        const double raw_confidence = candidate[8]; // 第8列为未归一化的总体置信度
        if (raw_confidence < raw_confidence_threshold)
            continue; // 先过滤低置信度候选，避免无效的类别比较

        // 类别分支得分：[13,21]分别对应9个类别
        int class_id = 0;
        double best_class_score = candidate[13];
        for (int class_index = 1; class_index < 9; ++class_index)
        {
            const double class_score = candidate[13 + class_index];
            if (class_score > best_class_score)
            {
                best_class_score = class_score;
                class_id = class_index;
            }
        }

        // sigmoid 还原 0-1 区间置信度
        const double confidence = raw_confidence >= 0.0
                                      ? 1.0 / (1.0 + std::exp(-raw_confidence))
                                      : std::exp(raw_confidence) / (1.0 + std::exp(raw_confidence));

        // 颜色分支得分：[9,12]分别对应4种颜色
        int color_id = 0;
        double best_color_score = candidate[9];
        for (int color_index = 1; color_index < 4; ++color_index)
        {
            const double color_score = candidate[9 + color_index];
            if (color_score > best_color_score)
            {
                best_color_score = color_score;
                color_id = color_index;
            }
        }

        if (color_id == 3)
            continue; // 忽略紫色装甲板（异常装甲板）
        if (color_id == 0 && my_color == 1)
            continue; // 己方为蓝色时，忽略蓝色装甲板（友军标识）
        if (color_id == 1 && my_color == 0)
            continue; // 己方为红色时，忽略红色装甲板（友军标识）

        std::vector<cv::Point2f> current_keypoints(4);

        for (int point_index = 0; point_index < 4; ++point_index)
        {
            const float x = candidate[point_index * 2 + 0];
            const float y = candidate[point_index * 2 + 1];

            // 还原到原图尺度
            const float kpt_x = (x - infer_param.pad_x) / infer_param.scale;
            const float kpt_y = (y - infer_param.pad_y) / infer_param.scale;

            current_keypoints[point_index] = cv::Point2f(kpt_x, kpt_y);
        }

        // 根据类别ID判断装甲板尺寸：1（英雄）和7（基地）为大装甲板，其他为小装甲板
        if (class_id == 1 || class_id == 7)
            sizes_temp.emplace_back(1); // 大装甲板标记
        else
            sizes_temp.emplace_back(0); // 小装甲板标记

        for (const cv::Point2f &point : current_keypoints)
            keypoints_temp.emplace_back(point.x, point.y);

        // 保存检测结果的各个分量
        confidences_temp.emplace_back(static_cast<float>(confidence));
        colors_temp.emplace_back(color_id);
        class_ids_temp.emplace_back(class_id);
        boxes_temp.emplace_back(cv::boundingRect(current_keypoints));
    }

    // 对候选框执行非极大值抑制
    cv::dnn::NMSBoxes(boxes_temp, confidences_temp, score_threshold, nms_threshold, indices_temp);

    // 组装非极大值抑制保留的装甲板结果
    for (const int &index : indices_temp)
    {
        // 准备结果
        NetArmorResult result;

        // 检查并存储关键点
        bool throw_flag = false;
        for (int point_index = 0; point_index < 4; ++point_index)
        {
            // 检查坐标是否超出模型输入范围，防止后续计算出错
            if (keypoints_temp[index * 4 + point_index].x > infer_param.origin_width ||
                keypoints_temp[index * 4 + point_index].x < 0 ||
                keypoints_temp[index * 4 + point_index].y > infer_param.origin_height ||
                keypoints_temp[index * 4 + point_index].y < 0)
            {
                throw_flag = true;
                break;
            }
        }
        if (throw_flag)
            continue; // 坐标无效，跳过此检测框

        result.points = {
            cv::Point2d(keypoints_temp[index * 4 + 0].x, keypoints_temp[index * 4 + 0].y),
            cv::Point2d(keypoints_temp[index * 4 + 1].x, keypoints_temp[index * 4 + 1].y),
            cv::Point2d(keypoints_temp[index * 4 + 2].x, keypoints_temp[index * 4 + 2].y),
            cv::Point2d(keypoints_temp[index * 4 + 3].x, keypoints_temp[index * 4 + 3].y)};

        // 存储类别
        result.armor_id = class_ids_temp[index];
        result.color_id = colors_temp[index];
        result.size = sizes_temp[index];
        result.score = static_cast<double>(confidences_temp[index]);
        result.class_name = m_armor_names[result.armor_id];
        result.color_name = m_color_names[result.color_id];

        // 对白色装甲板进行的额外判断（非红蓝击打变化为白色时不输出）
        if (!shouldAppendArmorResult(result, my_color, m_last_results))
            continue;

        // 相当于push_back，但是更省空间，不用拷贝
        results.emplace_back(std::move(result));
    }

    // 更新白色装甲板判定使用的历史结果
    m_last_results.pop();
    m_last_results.push(results);
    return results;
}

// ==================== v8步兵实现 ====================
// 【本工程不使用】对应输出 [25, 6300]，是 21 维那套的"带关键点置信度"版本：
// 每个关键点占 3 个数 (x, y, 关键点置信度)，所以 13 + i*3 而不是 13 + i*2，
// 总行数 25 = 4(颜色) + 9(兵种) + 4*3(四个关键点)
// ★ 本后处理【没有读那个关键点置信度】—— 解出来的第三个分量直接丢掉，
//   只是偏移量不同。所以 21 维和 25 维的差别实质只是"每点占几位"
// 初始化 V8 步兵后处理器及装甲板历史队列
V8InfantryPostProcessor::V8InfantryPostProcessor(const YOLOModel::ModelConfig &model_config, float nms_threshold)
    : PostProcessor(model_config, nms_threshold), m_last_results(buildInitialArmorHistory()) {}

// ==================== v8 21维步兵实现 ====================
// ★★★ 本工程实际执行的就是这一节 ★★★
//
// 初始化 V8 21维步兵后处理器及装甲板历史队列
// nms_threshold 由 YOLOModel::defaultNmsThreshold() 硬编码传入，
// armor 系列 = 0.2f，本类的代码无法修改它、也不想改
V8_21InfantryPostProcessor::V8_21InfantryPostProcessor(const YOLOModel::ModelConfig &model_config, float nms_threshold)
    : PostProcessor(model_config, nms_threshold), m_last_results(buildInitialArmorHistory()) {}

// 解析 V8 21维步兵模型输出并生成装甲板检测结果
//
// ============================ 输入 ============================
// input_ptr        推理引擎输出的 f32 裸指针，形状 [1, 21, 6300]
//                  ★ 不要当二维数组读！21 是特征维、6300 是候选维，
//                    候选 c 的特征 f 在 data[f * 6300 + c]（列优先视角），
//                    所以下面 CandidateView 传的是 stride=1 / feature_stride=6300
// infer_param      原图尺寸和缩放参数，用来还原关键点坐标
//                  out_tensor_rows 必须是 21（形状校验），out_tensor_cols 是 6300
// my_color         敌我标识，0=我方红、1=我方蓝、2=两边都留（本工程固定传 2）
//
// ============================ 21 行的含义 ============================
//   [0 : 4]   颜色 4 类：0 蓝 / 1 红 / 2 白 / 3 紫
//   [4 : 13]  兵种 9 类：0 哨兵 1 英雄 2 工程 3/4/5 号步兵
//                        6 前哨站 7 基地(底部) 8 基地(顶部)
//   [13: 21]  4 个关键点，每点只有 (x,y)，共 8 个数；
//             顺序固定为 左上 → 左下 → 右下 → 右上
//
// ============================ 输出 ============================
// std::vector<NetArmorResult>，points 已经是【原图像素坐标】，
// 顺序保持"左上 左下 右下 右上"（本函数不做重排，重排由调用方 Detector 做）
// 无有效结果时返回空 vector，不抛异常
//
// ============================ 边界条件 ============================
// - input_ptr 为空 / rows != 21：打 stderr 并返回空 vector（不是抛异常！）
// - 关键点还原后超出原图范围（<0 或 >origin_w/h）→ 整个框丢弃，不算结果
// - 候选列表为空时仍然会调用 NMSBoxes，OpenCV 对空输入返回空索引，安全
//
// ============================ 本工程的调用方 ============================
// src/detector.cpp 的 Detector::detect → 再按 [0,3,2,1] 重排 points、
// 丢掉非红蓝的 color_id、把 armor_id 映射成 ArmorName
std::vector<NetArmorResult> V8_21InfantryPostProcessor::postProcessArmorMat(const float *input_ptr, const InferParam &infer_param, const int &my_color)
{
    const float confidence_threshold = m_model_config.confidence_threshold;
    const float nms_threshold = m_nms_threshold;

    // 形状校验：4(颜色) + 9(兵种) + 4*2(四个关键点 xy) = 21
    // ★ 只校验了行数 21，没有校验列数 6300 —— 列数靠 CandidateView 的
    //   feature_stride 隐式使用（见下），万一是别的列数不会在这里被拦住
    if (!input_ptr || infer_param.out_tensor_rows != (4 + 9 + 4 * 2))
    {
        std::cerr << "模型输出形状不匹配，请检查模型路径" << std::endl;
        return {};
    }

    std::vector<int> class_ids_temp;
    std::vector<float> confidences_temp;
    std::vector<int> sizes_temp;
    std::vector<int> colors_temp;
    std::vector<cv::Rect> boxes_temp;
    std::vector<cv::Point2d> keypoints_temp;
    std::vector<int> indices_temp;
    std::vector<NetArmorResult> results;

    // 传给 NMSBoxes 的置信度门限就是构造时那个 confidence_threshold
    // （本工程 config.yaml 的 score_threshold，0.5）
    const float score_threshold = confidence_threshold;

    // V8 21维输出为 21*6300，每列是一组候选结果
    // ★ 注意循环变量走的是 cols(6300) 而不是 rows(21) —— 因为候选在列上
    //   这和 V5 那边"每行一个候选"正好相反，是本后处理和 V5 最大的结构差异
    for (int candidate_index = 0; candidate_index < infer_param.out_tensor_cols; ++candidate_index)
    {
        // 封装候选项，便于后续直接当作张量计算
        // ★ 参数顺序是 {数据, 候选下标, 候选步长, 特征步长}：
        //   candidate_stride = 1、feature_stride = 6300 表示"列优先"布局，
        //   即候选 c、特征 f 位于 data[f * 6300 + c]
        //   如果哪天模型输出改成 [1,6300,21]（转置），这里要跟着换成
        //   candidate_stride = 21、feature_stride = 1，否则读出来全是错的
        const CandidateView candidate{input_ptr, static_cast<std::size_t>(candidate_index), 1,
                                      static_cast<std::size_t>(infer_param.out_tensor_cols)};

        // 类别分支得分：[4,12]分别对应9个类别
        // 只做 argmax，不额外乘"目标置信度"—— 21 维输出里没有 obj 分支，
        // 类别分本身就是最终置信度（这也是它和 v5 的一个区别）
        int class_id = 0;
        double best_class_score = candidate[4];
        for (int class_index = 1; class_index < 9; ++class_index)
        {
            const double class_score = candidate[4 + class_index];
            if (class_score > best_class_score)
            {
                best_class_score = class_score;
                class_id = class_index;
            }
        }

        // ★ 这里【没有】sigmoid：直接拿类别分的 argmax 和 0.5 比
        //   前提是模型的 ONNX 图里已经把 sigmoid 烘进去了，输出本身就是 0~1
        //   （对比 V5 那边要先做反 sigmoid 才敢比，两套模型的约定不一样。）
        //   换模型时如果发现置信度全在 0 附近或全都 >1，先怀疑这一条假设不成立
        const double confidence = best_class_score;
        if (confidence < confidence_threshold)
            continue;

        // 颜色分支得分：[0,3]分别对应4种颜色
        // 同样是 argmax 取到 color_id：0 蓝 1 红 2 白 3 紫
        // ★ 颜色和兵种是两个【独立】的分支（不是联合分布），
        //   所以理论上可能出现"兵种是英雄、颜色是紫"这种组合，
        //   下面的过滤各自独立生效
        int color_id = 0;
        double best_color_score = candidate[0];
        for (int color_index = 1; color_index < 4; ++color_index)
        {
            const double color_score = candidate[color_index];
            if (color_score > best_color_score)
            {
                best_color_score = color_score;
                color_id = color_index;
            }
        }

        // 过滤异常颜色和己方颜色
        // 这三行就是 my_color 在 V8_21 后处理里的全部作用（另外还有一处白板历史判定）：
        if (color_id == 3)
            continue;                    // 紫色：永远丢，异常装甲板，和 my_color 无关
        if (color_id == 0 && my_color == 1)
            continue;                    // 蓝色板 + 我方蓝 → 友军，丢
        if (color_id == 1 && my_color == 0)
            continue;                    // 红色板 + 我方红 → 友军，丢
        // my_color == 2（本工程）时后两个条件恒不成立 → 红蓝都留，交给上层筛

        // 解码装甲板关键点并还原到原图尺度
        // 关键点区起点是 13，每个点占 2 个数（只有 x,y，没有关键点置信度）
        // 顺序：point_index 0/1/2/3 = 左上 / 左下 / 右下 / 右上（训练时定死）
        std::vector<cv::Point2f> current_keypoints(4);
        for (int point_index = 0; point_index < 4; ++point_index)
        {
            const float x = candidate[13 + point_index * 2];
            const float y = candidate[13 + point_index * 2 + 1];

            // ★ 模型坐标 → 原图坐标：先减 padding 偏移，再除以缩放系数
            //   纯 resize 无 padding 时 pad_x/pad_y 为 0，这一步退化成 x / scale
            //   顺序不能反（必须先减后除），否则 pad 会被放大 scale 倍
            const float kpt_x = (x - infer_param.pad_x) / infer_param.scale;
            const float kpt_y = (y - infer_param.pad_y) / infer_param.scale;

            current_keypoints[point_index] = cv::Point2f(kpt_x, kpt_y);
        }

        // 根据类别确定装甲板尺寸
        // size 不是网络的输出，是按兵种推的：英雄(1)和基地底部(7)挂大板，
        // 其余（含基地顶部 8）按小板处理。判错了会影响 Solver 的 PnP 尺寸假设
        if (class_id == 1 || class_id == 7)
            sizes_temp.emplace_back(1);
        else
            sizes_temp.emplace_back(0);

        for (const cv::Point2f &point : current_keypoints)
            keypoints_temp.emplace_back(point.x, point.y);

        // 保存候选结果的各个分量
        confidences_temp.emplace_back(static_cast<float>(confidence));
        colors_temp.emplace_back(color_id);
        class_ids_temp.emplace_back(class_id);
        boxes_temp.emplace_back(cv::boundingRect(current_keypoints));
    }

    // 对候选框执行非极大值抑制
    // 两个阈值：score_threshold 是置信度门限（来自 config），
    // nms_threshold 是 IOU 门限（来自 defaultNmsThreshold()，本工程 0.2f，不可配）
    // ★ 注意 boxes_temp 用的是四个关键点算出来的【最小外接矩形】，
    //   不是"轴对齐的检测框"——因为这些模型输出的是四点而不是 xywh
    //   斜着看的装甲板，这个矩形的面积会明显偏大，NMS 行为会因此偏"宽容"
    // ★ 这里没有做"按类别分组 NMS"，红蓝板、不同兵种之间会互相抑制
    //   打符那套才做了分组（见 centerDistanceNMS）
    cv::dnn::NMSBoxes(boxes_temp, confidences_temp, score_threshold, nms_threshold, indices_temp);

    // 组装非极大值抑制保留的装甲板结果
    for (const int &index : indices_temp)
    {
        NetArmorResult result;

        // 关键点越界检查：还原回原图后只要有任何一个点落在画面外，整条结果丢掉
        // ★ 这是"宁缺勿错"的过滤：宁可少报一个目标，也不给上层一个坐标越界的框
        //   （越界的框会让 Solver 解出离谱的位姿）
        // ★ 用 continue 而不是真的抛异常，尽管变量名叫 throw_flag
        bool throw_flag = false;
        for (int point_index = 0; point_index < 4; ++point_index)
        {
            if (keypoints_temp[index * 4 + point_index].x > infer_param.origin_width ||
                keypoints_temp[index * 4 + point_index].x < 0 ||
                keypoints_temp[index * 4 + point_index].y > infer_param.origin_height ||
                keypoints_temp[index * 4 + point_index].y < 0)
            {
                throw_flag = true;
                break;
            }
        }
        if (throw_flag)
            continue;

        // points 原样填四个点，顺序保持 左上 左下 右下 右上，
        // 本函数【不做重排】，需要在 PnP 前换顺序的话是调用方的事
        // （本工程在 src/detector.cpp 里按 [0,3,2,1] 重排）
        result.points = {
            cv::Point2d(keypoints_temp[index * 4 + 0].x, keypoints_temp[index * 4 + 0].y),
            cv::Point2d(keypoints_temp[index * 4 + 1].x, keypoints_temp[index * 4 + 1].y),
            cv::Point2d(keypoints_temp[index * 4 + 2].x, keypoints_temp[index * 4 + 2].y),
            cv::Point2d(keypoints_temp[index * 4 + 3].x, keypoints_temp[index * 4 + 3].y)};

        // 填结果。class_name/color_name 是直接用 armor_id/color_id 当下标查表，
        // 表长分别是 9 和 4，正好覆盖 0~8 和 0~3（紫色已在上面被滤掉），不会越界
        result.armor_id = class_ids_temp[index];
        result.color_id = colors_temp[index];
        result.size = sizes_temp[index];
        result.score = static_cast<double>(confidences_temp[index]);
        result.class_name = m_armor_names[result.armor_id];
        result.color_name = m_color_names[result.color_id];

        // 白板的历史判定。本工程 my_color=2，这个函数恒返回 true，
        // 所以这里实际上不会 continue；白/紫的过滤由调用方 Detector 做
        if (!shouldAppendArmorResult(result, my_color, m_last_results))
            continue;

        results.emplace_back(std::move(result));
    }

    // 更新白色装甲板判定使用的历史结果
    // ★ 存进去的是【已经过滤完的 results】，不是原始候选 —— 所以历史里不会有
    //   被丢掉的紫板/友军板，白板判定用的是"上一帧真正会输出的东西"
    // ★ 这一步必须在函数末尾、在 results 被返回之前做，且不能提前 return
    //   （前面那些 return {} 的早退分支都会跳过这次更新，等于历史停滞一帧 ——
    //    上游如此，对结果影响很小，不改）
    m_last_results.pop();
    m_last_results.push(results);
    return results;
}

// ==================== v8步兵实现 ====================
// 解析 V8 步兵模型输出并生成装甲板检测结果
std::vector<NetArmorResult> V8InfantryPostProcessor::postProcessArmorMat(const float *input_ptr, const InferParam &infer_param, const int &my_color)
{
    const float confidence_threshold = m_model_config.confidence_threshold;
    const float nms_threshold = m_nms_threshold;

    if (!input_ptr || infer_param.out_tensor_rows != (4 + 9 + 4 * 3))
    {
        std::cerr << "模型输出形状不匹配，请检查模型路径" << std::endl;
        return {};
    }
    std::vector<int> class_ids_temp;         // 装甲板类别ID
    std::vector<float> confidences_temp;     // 检测置信度（用于NMS）
    std::vector<int> sizes_temp;             // 装甲板尺寸标志（0=小装甲板，1=大装甲板）
    std::vector<int> colors_temp;            // 装甲板颜色类别（0=蓝，1=红，2=白，3=紫）
    std::vector<cv::Rect> boxes_temp;        // 边界框（用于OpenCV NMS算法）
    std::vector<cv::Point2d> keypoints_temp; // 装甲板四个关键点坐标
    std::vector<int> indices_temp;           // NMS后保留的检测框索引结果
    std::vector<NetArmorResult> results;     // 完整的装甲板检测结果
    const float score_threshold = confidence_threshold;

    // V8 25维输出为 25*6300，每列是一组候选结果
    for (int candidate_index = 0; candidate_index < infer_param.out_tensor_cols; ++candidate_index)
    {
        // 封装候选项，便于后续直接当作张量计算
        const CandidateView candidate{input_ptr, static_cast<std::size_t>(candidate_index), 1,
                                      static_cast<std::size_t>(infer_param.out_tensor_cols)};

        // 类别分支得分：[4,12]分别对应9个类别
        int class_id = 0;
        double best_class_score = candidate[4];
        for (int class_index = 1; class_index < 9; ++class_index)
        {
            const double class_score = candidate[4 + class_index];
            if (class_score > best_class_score)
            {
                best_class_score = class_score;
                class_id = class_index;
            }
        }

        const double confidence = best_class_score;
        if (confidence < confidence_threshold)
            continue; // 置信度低于阈值的直接跳过

        // 颜色分支得分：[0,3]分别对应4种颜色
        int color_id = 0;
        double best_color_score = candidate[0];
        for (int color_index = 1; color_index < 4; ++color_index)
        {
            const double color_score = candidate[color_index];
            if (color_score > best_color_score)
            {
                best_color_score = color_score;
                color_id = color_index;
            }
        }

        if (color_id == 3)
            continue; // 忽略紫色装甲板（异常装甲板）
        if (color_id == 0 && my_color == 1)
            continue; // 己方为蓝色时，忽略蓝色装甲板（友军标识）
        if (color_id == 1 && my_color == 0)
            continue; // 己方为红色时，忽略红色装甲板（友军标识）

        std::vector<cv::Point2f> current_keypoints(4);
        for (int point_index = 0; point_index < 4; ++point_index)
        {
            const float x = candidate[13 + point_index * 3 + 0];
            const float y = candidate[13 + point_index * 3 + 1];

            // 还原到原图尺度
            const float kpt_x = (x - infer_param.pad_x) / infer_param.scale;
            const float kpt_y = (y - infer_param.pad_y) / infer_param.scale;

            current_keypoints[point_index] = cv::Point2f(kpt_x, kpt_y);
        }

        // 根据类别ID判断装甲板尺寸：1（英雄）和7（基地）为大装甲板，其他为小装甲板
        if (class_id == 1 || class_id == 7)
            sizes_temp.emplace_back(1); // 大装甲板标记
        else
            sizes_temp.emplace_back(0); // 小装甲板标记

        for (const cv::Point2f &point : current_keypoints)
            keypoints_temp.emplace_back(point.x, point.y);

        // 保存检测结果的各个分量
        confidences_temp.emplace_back(static_cast<float>(confidence));
        colors_temp.emplace_back(color_id);
        class_ids_temp.emplace_back(class_id);
        boxes_temp.emplace_back(cv::boundingRect(current_keypoints));
    }

    // 对候选框执行非极大值抑制
    cv::dnn::NMSBoxes(boxes_temp, confidences_temp, score_threshold, nms_threshold, indices_temp);

    // 组装非极大值抑制保留的装甲板结果
    for (const int &index : indices_temp)
    {
        // 准备结果
        NetArmorResult result;

        // 检查并存储关键点
        bool throw_flag = false;
        for (int point_index = 0; point_index < 4; ++point_index)
        {
            // 检查坐标是否超出模型输入范围，防止后续计算出错
            if (keypoints_temp[index * 4 + point_index].x > infer_param.origin_width ||
                keypoints_temp[index * 4 + point_index].x < 0 ||
                keypoints_temp[index * 4 + point_index].y > infer_param.origin_height ||
                keypoints_temp[index * 4 + point_index].y < 0)
            {
                throw_flag = true;
                break;
            }
        }
        if (throw_flag)
            continue; // 坐标无效，跳过此检测框

        result.points = {
            cv::Point2d(keypoints_temp[index * 4 + 0].x, keypoints_temp[index * 4 + 0].y),
            cv::Point2d(keypoints_temp[index * 4 + 1].x, keypoints_temp[index * 4 + 1].y),
            cv::Point2d(keypoints_temp[index * 4 + 2].x, keypoints_temp[index * 4 + 2].y),
            cv::Point2d(keypoints_temp[index * 4 + 3].x, keypoints_temp[index * 4 + 3].y)};

        // 存储类别
        result.armor_id = class_ids_temp[index];
        result.color_id = colors_temp[index];
        result.size = sizes_temp[index];
        result.score = static_cast<double>(confidences_temp[index]);
        result.class_name = m_armor_names[result.armor_id];
        result.color_name = m_color_names[result.color_id];

        if (!shouldAppendArmorResult(result, my_color, m_last_results))
            continue;

        // 相当于push_back，但是更省空间，不用拷贝
        results.emplace_back(std::move(result));
    }

    // 更新白色装甲板判定使用的历史结果
    m_last_results.pop();
    m_last_results.push(results);
    return results;
}

// ==================== 雷达实现 ====================
// 【本工程不使用】雷达站视角的模型，输出 [26, 2100]，上游唯一真正用 TensorRT 的兵种
// 和其他装甲板后处理的两点不同：
//   1) 有独立的 obj 框：前 4 个数是锚框 cx,cy,w,h，NMS 用的是【真锚框】而不是
//      关键点外接矩形；类别从第 4 个数开始共 10 类
//   2) 颜色不是独立分支，而是编码进类别里（0~4 蓝，5~9 红），
//      所以 result.color_id = armor_id / 5。也正因此它忽略 my_color
//      （函数开头的 static_cast<void>(my_color) 就是明示"这个参数用不上"）
//      把类别和颜色揉在一起，是雷达只需要报"哪台车"、不需要区分敌我的场景使然
// 初始化雷达后处理器
LidarPostProcessor::LidarPostProcessor(const YOLOModel::ModelConfig &model_config, float nms_threshold)
    : PostProcessor(model_config, nms_threshold) {}

// 解析雷达模型输出并生成装甲板检测结果
std::vector<NetArmorResult> LidarPostProcessor::postProcessArmorMat(const float *input_ptr, const InferParam &infer_param, const int &my_color)
{
    static_cast<void>(my_color);
    const float confidence_threshold = m_model_config.confidence_threshold;
    const float nms_threshold = m_nms_threshold;

    if (!input_ptr || infer_param.out_tensor_rows != (4 + 10 + 4 * 3))
    {
        std::cerr << "模型输出形状不匹配，请检查模型路径" << std::endl;
        return {};
    }
    std::vector<int> class_ids_temp;         // 装甲板类别ID
    std::vector<float> confidences_temp;     // 检测置信度（用于NMS）
    std::vector<cv::Rect> rects_temp;        // 雷达中rect由原始的xywh输出
    std::vector<cv::Point2d> keypoints_temp; // 装甲板四个关键点坐标
    std::vector<NetArmorResult> results;     // 完整的装甲板检测结果

    // 雷达网络输出形状为 26*2100，每列是一个预测结果
    for (int current_anchor_box = 0; current_anchor_box < infer_param.out_tensor_cols; current_anchor_box++)
    {
        // 封装候选项，便于后续直接当作张量计算
        const CandidateView candidate{input_ptr, static_cast<std::size_t>(current_anchor_box), 1,
                                      static_cast<std::size_t>(infer_param.out_tensor_cols)};

        // 置信度最高的类别的索引和置信度
        int best_class_idx = 0;
        double max_class_conf = candidate[4];
        for (int cls = 1; cls < 10; ++cls)
        {
            double score = candidate[4 + cls];
            if (score > max_class_conf)
            {
                max_class_conf = score;
                best_class_idx = cls;
            }
        }

        // 如果最大的置信度也低于阈值,则跳过该锚框
        if (max_class_conf < confidence_threshold)
        {
            continue;
        }

        // 前四行是锚框的位置和大小
        float cx_temp = candidate[0];
        float cy_temp = candidate[1];
        float w_temp = candidate[2];
        float h_temp = candidate[3];

        // 还原到原图尺度
        int lt_x = std::max(0.f, (((cx_temp - 0.5f * w_temp) - infer_param.pad_x) / infer_param.scale) + 0.5f); // lt_x为缩放前原图片中锚框的左上角x值
        int lt_y = std::max(0.f, (((cy_temp - 0.5f * h_temp) - infer_param.pad_y) / infer_param.scale) + 0.5f);
        int w = static_cast<int>(w_temp / infer_param.scale + 0.5); // 缩放前的锚框宽度
        int h = static_cast<int>(h_temp / infer_param.scale + 0.5);

        // 放入容器
        // best_class_idx是1*10向量中最大的点，x为1，y是最大config值
        class_ids_temp.push_back(best_class_idx);
        confidences_temp.push_back(static_cast<float>(max_class_conf));
        rects_temp.push_back(cv::Rect(lt_x, lt_y, w, h));

        // keypoints解码(4个关键点)
        for (int kpt_num = 0; kpt_num < 4; kpt_num++)
        {
            float kpt_x_temp = candidate[14 + kpt_num * 3 + 0]; // x
            float kpt_y_temp = candidate[14 + kpt_num * 3 + 1]; // y

            // 还原到原图尺度
            float kpt_x = (kpt_x_temp - infer_param.pad_x) / infer_param.scale;
            float kpt_y = (kpt_y_temp - infer_param.pad_y) / infer_param.scale;

            // 放入容器
            keypoints_temp.push_back(cv::Point2d(kpt_x, kpt_y));
        }
    }

    // 只要任何一个容器为空,说明没有结果（这四个容器是一起传数据的，所以顺序是固定的）
    if (class_ids_temp.empty())
    {
        return {};
    }

    // 非极大值抑制：将锚框和其对应的置信度作非极大值抑制去重，阈值为置信度和交并比，输出参数为保留的索引（indices）
    std::vector<int> indices; // 索引容器
    cv::dnn::NMSBoxes(rects_temp, confidences_temp, confidence_threshold, nms_threshold, indices);

    // 遍历indices并且处理偏移来生成最终的返回值
    results.reserve(indices.size());
    for (const int &index : indices)
    {
        // 准备结果
        NetArmorResult result;

        // 检查并存储关键点
        bool throw_flag = false;
        for (int i = 0; i < 4; ++i)
        {
            // 检查坐标是否超出模型输入范围，防止后续计算出错
            if (keypoints_temp[index * 4 + i].x > infer_param.origin_width ||
                keypoints_temp[index * 4 + i].x < 0 ||
                keypoints_temp[index * 4 + i].y > infer_param.origin_height ||
                keypoints_temp[index * 4 + i].y < 0)
            {
                throw_flag = true;
                break;
            }
        }
        if (throw_flag)
            continue; // 坐标无效，跳过此检测框

        result.points = {
            cv::Point2d(keypoints_temp[index * 4 + 0].x, keypoints_temp[index * 4 + 0].y),
            cv::Point2d(keypoints_temp[index * 4 + 1].x, keypoints_temp[index * 4 + 1].y),
            cv::Point2d(keypoints_temp[index * 4 + 2].x, keypoints_temp[index * 4 + 2].y),
            cv::Point2d(keypoints_temp[index * 4 + 3].x, keypoints_temp[index * 4 + 3].y)};

        // 存储类别
        result.armor_id = class_ids_temp[index];
        result.score = static_cast<double>(confidences_temp[index]);
        result.color_id = result.armor_id / 5;
        // 雷达装甲板不需要size属性
        result.size = 0;
        result.class_name = m_armor_names[result.armor_id];
        result.color_name = m_color_names[result.color_id];

        // 相当于push_back，但是更省空间，不用拷贝
        results.emplace_back(std::move(result));
    }

    return results;
}

// ==================== 符实现 ====================
// 【本工程不使用】打符模型，输出 [18, 6300]：3(未激活/小符/大符三个类别) +
// 5*3(五个关键点，每点带 x,y,置信度)。5 个关键点的顺序是
// top、left、R(旋转中心)、right、bottom —— 第 3 个点是中心而不是第 5 个
// 它和装甲板后处理有两处结构性差异：
//   1) NMS 不用 IOU 而用中心点像素距离（centerDistanceNMS），阈值 100 像素；
//   2) 要求至少 3 个关键点的置信度 > 0.8 才算有效检测
// 初始化神符后处理器
RunePostProcessor::RunePostProcessor(const YOLOModel::ModelConfig &model_config, float nms_threshold)
    : PostProcessor(model_config, nms_threshold) {}

namespace
{
// 按符叶四个边缘点的中心距离执行非极大值抑制
//
// 为什么不用 IOU：符叶是近似圆形/扇形的目标，而且同一块符叶会因为旋转被网络
// 多次响应，"框"本身也不稳；用中心点距离更符合"两个检测是不是同一块符叶"
//
// center_dist_threshold 是【像素距离】（默认 100，来自 defaultNmsThreshold 的
// rune 分支），和 armor 那边的 IOU 阈值是两种量纲，别互相对照
// 分组规则：class_id < 2（未激活 + 小符）算一组，class_id == 2（大符）单独一组，
// 不同组之间不互相抑制 —— 因为同一个位置的大符和小符本来就该同时存在
// 大符那一组的距离阈值是标准值的 1/3（代码里用平方除 9 实现，避免开方）
void centerDistanceNMS(
    const std::vector<cv::Point2d> &keypoints_list, // 每个关键点: (x, y)
    const std::vector<float> &confidences_list,
    const std::vector<int> &class_ids_list,
    float center_dist_threshold,
    std::vector<int> &indices)
{
    indices.clear();

    // 检查输入
    if (keypoints_list.empty() || confidences_list.empty() || class_ids_list.empty())
    {
        return;
    }

    // 每个检测应该有5个关键点
    const int POINTS_PER_DETECTION = 5;
    int num_detections = confidences_list.size();

    if (keypoints_list.size() != static_cast<std::size_t>(num_detections * POINTS_PER_DETECTION))
    {
        std::cerr << "NMS时关键点数目错误" << std::endl;
        return;
    }

    // 1. 计算每个检测的中心点（使用符叶4个角点的平均值）
    std::vector<cv::Point2f> centers;
    centers.reserve(num_detections);

    for (int i = 0; i < num_detections; ++i)
    {
        // 获取当前检测的5个关键点
        const cv::Point2d *kpts = &keypoints_list[i * POINTS_PER_DETECTION];

        // 跳过索引2的R点，使用 top、left、right、bottom 四点计算中心
        cv::Point2f center(0, 0);
        for (int k = 0; k < 5; ++k)
        {
            // 跳过R标
            if (k == 2)
                continue;
            center.x += kpts[k].x;
            center.y += kpts[k].y;
        }
        center.x /= 4.0f;
        center.y /= 4.0f;
        centers.push_back(center);
    }
    // 2. 初始化抑制标记
    std::vector<bool> suppressed(num_detections, false);

    // 3. 距离阈值平方（避免开方计算）
    float dist_threshold_sq = center_dist_threshold * center_dist_threshold;

    // 4. 按类别分组，但保持原indices不变，只给每个小符和大符打标签
    std::vector<int> sorted_indices(num_detections);
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);

    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&confidences_list](int a, int b)
              {
                  return confidences_list[a] > confidences_list[b];
              });

    auto same_group = [&class_ids_list](int a, int b)
    {
        bool a_lt_2 = class_ids_list[a] < 2;
        bool b_lt_2 = class_ids_list[b] < 2;
        bool a_eq_2 = class_ids_list[a] == 2;
        bool b_eq_2 = class_ids_list[b] == 2;
        return (a_lt_2 && b_lt_2) || (a_eq_2 && b_eq_2);
    };

    // 5. 同组间按置信度降序排序（保持索引），不同组不比较
    for (size_t i = 0; i < sorted_indices.size(); ++i)
    {
        int idx_i = sorted_indices[i];

        if (suppressed[idx_i])
            continue;

        indices.push_back(idx_i);
        const cv::Point2f &center_i = centers[idx_i];

        for (size_t j = i + 1; j < sorted_indices.size(); ++j)
        {
            int idx_j = sorted_indices[j];

            if (suppressed[idx_j])
                continue;

            // 不在同一组，不能互相抑制
            if (!same_group(idx_i, idx_j))
                continue;

            const cv::Point2f &center_j = centers[idx_j];
            float dx = center_i.x - center_j.x;
            float dy = center_i.y - center_j.y;
            float dist_sq = dx * dx + dy * dy;

            // 大符的nms距离阈值是正常小符/未激活的1/3，这里平方了所以除9
            float current_dist_threshold_sq = dist_threshold_sq;
            if (class_ids_list[idx_i] == 2)
            {
                current_dist_threshold_sq = dist_threshold_sq / 9.0f;
            }

            if (dist_sq < current_dist_threshold_sq)
            {
                suppressed[idx_j] = true;
            }
        }
    }
}
} // namespace

// 解析神符模型输出并生成符叶检测结果
std::vector<NetRuneResult> RunePostProcessor::postProcessRuneMat(const float *input_ptr, const InferParam &infer_param)
{
    const float confidence_threshold = m_model_config.confidence_threshold;
    const float nms_threshold = m_nms_threshold;

    if (!input_ptr || infer_param.out_tensor_rows != (3 + 5 * 3))
    {
        std::cerr << "模型输出形状不匹配，请检查模型路径" << std::endl;
        return {};
    }

    std::vector<int> class_ids_temp;         // 符叶类别ID
    std::vector<float> confidences_temp;     // 检测置信度
    std::vector<cv::Point2d> keypoints_temp; // 符叶五个关键点坐标
    std::vector<int> indices_temp;           // NMS后保留的结果索引
    std::vector<NetRuneResult> results;      // 完整的符叶检测结果

    // 打符网络形状是 18*6300，每列是一个预测结果
    for (int candidate_index = 0; candidate_index < infer_param.out_tensor_cols; ++candidate_index)
    {
        // 封装候选项，便于后续直接当作张量计算
        const CandidateView candidate{input_ptr, static_cast<std::size_t>(candidate_index), 1,
                                      static_cast<std::size_t>(infer_param.out_tensor_cols)};

        // 选择置信度最高的符叶类别
        int class_id = 0;
        float best_class_score = candidate[0];
        for (int class_index = 1; class_index < 3; ++class_index)
        {
            const float class_score = candidate[class_index];
            if (class_score > best_class_score)
            {
                best_class_score = class_score;
                class_id = class_index;
            }
        }

        if (best_class_score < confidence_threshold)
            continue;

        // 统计达到置信度要求的关键点数量
        // 【移植改动 · 2026-10-04】阈值 0.8 → 0.5：0.8 是移植时照 tdt-campus-test
        // Python 抄的，在那台机器上够用；换 RP-26Rune 主算法后暴露出问题 ——
        // 本游戏里符叶检测本来就在 0.6~0.8 之间闪（远一点/暗一点就掉），
        // 0.8 门限把检测打成"碎片"（实测小符素材只有 62% 的帧有检出），
        // 而 相位滤波器 0.1s 没观测就清队列重建 —— 打不中的主因之一
        // 原版自己的检测配置就是 kconf=0.5（min 5 点），这里对齐到 0.5
        int valid_keypoint_count = 0;
        for (int point_index = 0; point_index < 5; ++point_index)
        {
            const float keypoint_confidence = candidate[3 + point_index * 3 + 2];
            if (keypoint_confidence > 0.5f)
                ++valid_keypoint_count;
        }

        if (valid_keypoint_count < 3)
            continue;

        // 解码符叶关键点并还原到原图尺度
        std::vector<cv::Point2d> current_keypoints(5);
        for (int point_index = 0; point_index < 5; ++point_index)
        {
            const float x = candidate[3 + point_index * 3 + 0];
            const float y = candidate[3 + point_index * 3 + 1];

            const float kpt_x = (x - infer_param.pad_x) / infer_param.scale;
            const float kpt_y = (y - infer_param.pad_y) / infer_param.scale;

            current_keypoints[point_index] = cv::Point2d(kpt_x, kpt_y);
        }

        for (const cv::Point2d &point : current_keypoints)
            keypoints_temp.emplace_back(point);

        // 保存候选结果的各个分量
        confidences_temp.emplace_back(best_class_score);
        class_ids_temp.emplace_back(class_id);
    }

    // 候选容器同步写入，类别为空表示没有有效结果
    if (class_ids_temp.empty())
        return {};

    // 非极大值抑制：使用的是符叶中心四点的xy平均值，即符叶中心点坐标的距离代替iou，nms阈值为30-100左右（原图像素尺寸）
    centerDistanceNMS(keypoints_temp, confidences_temp, class_ids_temp, nms_threshold, indices_temp);

    // 遍历NMS保留索引并生成最终结果
    results.reserve(indices_temp.size());
    for (const int &index : indices_temp)
    {
        // 准备结果
        NetRuneResult result;

        // 检查并存储关键点
        bool throw_flag = false;
        for (int point_index = 0; point_index < 5; ++point_index)
        {
            // 检查坐标是否超出模型输入范围，防止后续计算出错
            if (keypoints_temp[index * 5 + point_index].x > infer_param.origin_width ||
                keypoints_temp[index * 5 + point_index].x < 0 ||
                keypoints_temp[index * 5 + point_index].y > infer_param.origin_height ||
                keypoints_temp[index * 5 + point_index].y < 0)
            {
                throw_flag = true;
                break;
            }
        }
        if (throw_flag)
            continue; // 坐标无效，跳过此检测框

        result.points = {
            cv::Point2d(keypoints_temp[index * 5 + 0].x, keypoints_temp[index * 5 + 0].y),
            cv::Point2d(keypoints_temp[index * 5 + 1].x, keypoints_temp[index * 5 + 1].y),
            cv::Point2d(keypoints_temp[index * 5 + 2].x, keypoints_temp[index * 5 + 2].y),
            cv::Point2d(keypoints_temp[index * 5 + 3].x, keypoints_temp[index * 5 + 3].y),
            cv::Point2d(keypoints_temp[index * 5 + 4].x, keypoints_temp[index * 5 + 4].y)};
        result.top = result.points[0];
        result.left = result.points[1];
        result.point_R = result.points[2];
        result.right = result.points[3];
        result.bottom = result.points[4];

        // 存储类别
        result.class_id = class_ids_temp[index];
        result.score = confidences_temp[index];
        result.class_name = m_class_names[result.class_id];

        // 相当于push_back，但是更省空间，不用拷贝
        results.emplace_back(std::move(result));
    }

    return results;
}

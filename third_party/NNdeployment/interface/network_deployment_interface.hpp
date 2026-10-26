#pragma once

// ============================================================================
// network_deployment_interface.hpp —— 部署库对外的唯一头文件
//
// 职责：把 26_NNDeployment 那套统一部署库（预处理 / 推理 / 后处理三段）包装成
// 两个"只管喂图、只管出结果"的类：
//     ArmorModel → netProcess(img, my_color) → std::vector<NetArmorResult>
//     RuneModel  → netProcess(img)           → std::vector<NetRuneResult>
// 真正干活的 YOLOModel、推理引擎（TensorRT/OpenVINO）、各类 PostProcessor 全在
// lib 层，调用方看不到。集成方式：include 本头文件 + 链接 NNdeployment_lib
//
// ★ 本工程（tdt-learning4.4）只用 ArmorModel，且只走 TensorRT 后端：
//   见 src/detector.hpp / src/detector.cpp，Detector 持有 unique_ptr<ArmorModel>
//   RuneModel 这一半在本工程属于死代码，留着只是为了不破坏和上游的可比性
//
// ★ 本文件里的 struct 是"跨库契约"：上游一旦改字段顺序或语义，本工程
//   Detector 里的映射（颜色 → blue/red、armor_id → ArmorName、points 顺序重排）
//   就会静默出错。换库版本时优先复核这个文件
// ============================================================================

#include <opencv2/core.hpp>

#include <memory>
#include <string>
#include <vector>

class YOLOModel;

// JSON 配置入口所需的三个路径参数
//
// 注意三个字段拼起来的路径是怎么算的：model_folder / <节点里的 "xml" 字段>，
// 即"模型文件夹 + 文件名"，节点名和文件名是两个独立参数（见
// network_deployment_lib.cpp 的 loadModelConfigFromJson）
// 虽然字段叫 json，实际用的是 cv::FileStorage，OpenCV 的 YAML/JSON 都能吃
// TensorRT 后端下这个 "xml" 填的其实是 .engine 文件名
class JsonConfig
{
public:
    // json路径、json内部key、模型文件夹路径
    JsonConfig(std::string json_path,
               std::string model_key,
               std::string model_folder);

    std::string json_path;
    std::string model_key;
    std::string model_folder;
};

// 装甲板检测结果，坐标均为原图像素坐标
//
// ===== points 的顺序（最容易踩的坑）=====
// 顺序是网络训练时就定死的：左上 → 左下 → 右下 → 右上（逆时针绕一圈，
// 图像坐标系 y 轴向下）。部署库不会做任何重排，原样吐出来
// 而本工程 Solver::pnp_points() 需要的 3D 点顺序是 左上 右上 右下 左下（顺时针），
// 所以 Detector::detect() 里必须按 [0,3,2,1] 重排一次；漏了这一步，
// 画出来的四边框会变成"蝴蝶结"（对角线交叉），PnP 位姿也会跟着错
//
// ===== color_id 的取值 =====
// 是网络 4 分类 argmax 的结果：0 蓝 / 1 红 / 2 白 / 3 紫
//   3（紫，异常装甲板）在后处理里就被 continue 掉了，不会出现在结果里；
//   2（白）只有在 my_color==2，或者最近 5 帧历史里出现过"同类别 + 同位置(IOU>0.3)
//   的红/蓝板"时才会保留（见 network_postprocess.cpp 的 shouldAppendArmorResult）
//   本工程把 my_color 固定成 2，所以白板也会被返回，Detector 里再按
//   color_id != 0 && color_id != 1 主动丢弃
struct NetArmorResult
{
    std::vector<cv::Point2d> points; // 左上、左下、右下、右上
    int armor_id = -1;               // 兵种 9 分类：0 哨兵 1 英雄 2 工程 3/4/5 号步兵
                                     // 6 前哨站 7 基地(底部) 8 基地(顶部)
                                     // 顺序由 V8_21InfantryPostProcessor::m_armor_names 决定，
                                     // 本工程 Detector 的 kArmorNames[9] 必须和它逐个对齐
    int color_id = -1;               // 0=蓝色，1=红色，2=白色。my_color=1表示我方为蓝色，=0表示我方为红色
    int size = 0;                    // 0=小装甲板，1=大装甲板
                                     // 不是网络直接输出的量：后处理按 armor_id 推的
                                     // ——class_id 1(英雄) 和 7(基地底部) 算大板，其余算小板
    double score = 0.;               // 置信度。注意是"类别分支的 argmax 值"，不是 obj*cls
                                     // armor 后处理不做 sigmoid（见 postprocess 里的说明），
                                     // 这里假定该值已在 0~1 区间
    std::string class_name;          // 上面的 armor_id 对应的中文名，调试用
    std::string color_name;          // 上面的 color_id 对应的中文名，调试用
};

// 神符检测结果，坐标均为原图像素坐标
// 本工程不打符，这个结构只是跟着库一起搬过来的，没有使用方
// points 顺序为 top、left、R、right、bottom —— 第 3 个点是旋转中心 R，不是第 5 个，
// 顺序和装甲板的四点完全不同，将来要用得注意
struct NetRuneResult
{
    std::vector<cv::Point2d> points; // top、left、R、right、bottom
    cv::Point2d top, left, right, bottom, point_R;
    int class_id = -1;
    double score = 0.;
    std::string class_name;
};

// 控制模型信息输出与逐阶段耗时统计，可选构造参数之一
//
// print_debug_info：打开后 TensorRT 后端会多打一段"模型张量信息"
//   （输入输出张量的名字/形状/数据类型），排查"模型和代码对不上"时很有用
//   注意它由引擎实现自己判断，各后端打印内容不一样
// calculate_speed_info：打开后 YOLOModel 会创建 MPT::SpeedStats，
//   每帧统计 preprocess / inference / postprocess 三段的微秒耗时并打印
//   这个开关会真的走计时代码（steady_clock 调用 + 打印），
//   测速之外别开，免得白花时间
// 两个开关都在 YOLOModel 构造时读一次，之后不能改
struct DebugConfig
{
    bool print_debug_info = false;      // 开启后会输出网络的debug调试结果
    bool calculate_speed_info = false;  // 开启后会输出网络各阶段耗时统计信息

    DebugConfig(bool debug = false, bool speed = false)
        : print_debug_info(debug), calculate_speed_info(speed)
    {}
};

// 只产生装甲板结果的网络模型。对象不可复制，可以移动
//
// ★ 构造 = 加载模型 + 建引擎 + 挑后处理，全部在这一步完成，很重（读盘、反序列化、
//   cudaMalloc、建 CUDA 流都在里面），不要每帧建对象
// ★ 构造会抛异常：模型文件不存在/读不了、模型输出形状识别不了、
//   以及"模型后处理类型和类不匹配"（比如拿神符模型构造 ArmorModel）都会抛
//   所以本工程 Detector 的构造必须放在 try/catch 或让调用方兜底
// ★ 三个构造函数最终都收敛到内部的 YOLOModel(ModelConfig, DebugConfig)，
//   所以用哪个入口结果都一样，区别只是"参数从哪来"（写死 / 逐个传 / 读 JSON）
class ArmorModel
{
public:
    // model_path：模型文件完整路径
    // ★ 这个重载把其余参数全部写死成：async + openvino + GPU + 0.5 + auto_detect，
    //   也就是【默认走 OpenVINO】。本工程是 TensorRT 后端，千万别用这个重载
    explicit ArmorModel(const std::string &model_path);
    // 直接指定模型路径、推理模式和部署后端，剩余参数使用默认值
    //
    // 参数逐个说明（本工程 Detector 用的就是这个重载）：
    //   model_path           模型文件完整路径。TensorRT 后端下是 .engine 文件
    //                        （本工程配的是 ../models/infantry_v8n.engine）
    //                        engine 与 GPU 架构 / TensorRT 版本绑定，换机器就得重新生成
    //   infer_mode           "sync" / "async" / "async4"
    //                        ★ TensorRT 后端里 sync 和 async 走的是同一段代码
    //                          （TensorRTEngine::syncInfer 和 asyncInfer 逐行相同，
    //                          结尾都有 cudaStreamSynchronize，其实是"假异步"），
    //                          写哪个都行，本工程写 "sync" 只是名字更贴合真实行为
    //                        "async4" 在 TRT 后端没有实现，会抛 logic_error
    //   deploy_way           "openvino" / "tensorrt"。写错字符串直接抛 runtime_error
    //                        （不是静默回退）。本工程必须传 "tensorrt"
    //   device               OpenVINO 的设备名（"GPU"/"CPU"/"NPU"）
    //                        ★ TensorRT 后端完全忽略这个参数，传什么都行
    //   confidence_threshold 置信度阈值，透传给后处理做候选筛选
    //                        ★ 它不是 NMS 阈值，NMS 阈值在库里硬编码（见下文）
    //   postprocess_mode     后处理模式。默认 "auto_detect"：按引擎报出来的输出
    //                        张量形状自动挑后处理，[1,21,6300] 会命中
    //                        V8_21InfantryPostProcessor。本工程就用 auto_detect，
    //                        想在日志里明确看到选了什么也可以直接写 "v8infantry_21"
    //   debug_config         见 DebugConfig
    ArmorModel(const std::string &model_path,
               const std::string &infer_mode,
               const std::string &deploy_way,
               const std::string &device = "GPU",
               float confidence_threshold = 0.5f,
               const std::string &postprocess_mode = "auto_detect",
               const DebugConfig &debug_config = DebugConfig());
    // 从 JSON 的指定节点读取模型配置
    // ★ 读的字段只有 xml / infer_mode / deploy_way / postprocess_mode /
    //   device / score_threshold（兼容旧名 config_thresh）
    //   ★ config.yaml 里的 nms_threshold 不在其中 —— 库里根本没读它，
    //     所以那个配置项是失效的（NMS 阈值硬编码在 defaultNmsThreshold()）
    explicit ArmorModel(const JsonConfig &json_config,
                        const DebugConfig &debug_config = DebugConfig());
    ~ArmorModel() noexcept;

    // 禁止复制构造和赋值，允许移动构造和移动赋值
    ArmorModel(const ArmorModel &) = delete;
    ArmorModel &operator=(const ArmorModel &) = delete;
    ArmorModel(ArmorModel &&) noexcept;
    ArmorModel &operator=(ArmorModel &&) noexcept;

    // 执行装甲板推理；my_color=1 表示我方蓝色，=0 表示我方红色，=2 表示离线演示时保留蓝红双方结果
    // 无有效结果时返回空 vector；模型任务不匹配会在构造时抛出异常
    //
    // ===== my_color 到底做了什么 =====
    // 它只在后处理里当"敌我过滤开关"用，一共三处判断（以 21 维后处理为例）：
    //   color_id 是颜色分类结果：0 蓝 1 红 2 白 3 紫
    //   color_id==3        → 紫，永远丢弃（异常装甲板），和 my_color 无关
    //   color_id==0 && my_color==1 → 我方是蓝，丢蓝板（友军不打印）
    //   color_id==1 && my_color==0 → 我方是红，丢红板（友军不打印）
    //   白板(cid 2)另有历史判定：my_color==2 时全留，否则要看最近 5 帧里有没有
    //   同位置同兵种的红/蓝板（用来滤掉"打中后变白"的干扰）
    // 所以 my_color=2（本工程固定用的值）等于"不做敌我过滤，红蓝双方都返回"，
    // 敌我判断交给上层 main.cpp 按 enemy_color 做 —— 旧版也是这么分工的
    //
    // ===== 输入图像的要求 =====
    // 传 BGR 的 cv::Mat 即可（库内部会有 BGR→RGB 的换序），尺寸任意：
    // 内部做的是纯 resize、无 padding，关键点会按同样的比例还原回原图坐标
    // 但对 4:3 以外的图，因为模型输入 640×480 也是 4:3，会走 padding 分支
    // （填 124 灰），此时关键点还原要减掉 pad 偏移，库里已经处理好了
    std::vector<NetArmorResult> netProcess(const cv::Mat &input_image, const int &my_color);

private:
    // 具体实现由内部的 YOLOModel 完成，ArmorModel 仅作为接口类，对外暴露构造函数和armor获取函数
    // 每构造一个 ArmorModel 就是一份独立的引擎 + 一份独立的后处理状态
    // （后处理里有"最近 5 帧白板历史"队列，所以两个实例的历史互不影响）
    // 用 unique_ptr 而非值成员，是因为 YOLOModel 在头文件里只有前向声明
    std::unique_ptr<YOLOModel> m_model;
};

// 只产生神符结果的网络模型。对象不可复制，可以移动
// ★ 本工程不打符（比赛用的打符节点是另一个工程 tdt-campus-test），
//   这个类在 tdt-learning4.4 里没有被任何地方实例化
class RuneModel
{
public:
    // model_path：模型文件完整路径
    explicit RuneModel(const std::string &model_path);
    // 直接指定模型路径、推理模式和部署后端，剩余参数使用默认值
    RuneModel(const std::string &model_path,
              const std::string &infer_mode,
              const std::string &deploy_way,
              const std::string &device = "GPU",
              float confidence_threshold = 0.5f,
              const std::string &postprocess_mode = "auto_detect",
              const DebugConfig &debug_config = DebugConfig());
    // 从 JSON 的指定节点读取模型配置
    explicit RuneModel(const JsonConfig &json_config,
                       const DebugConfig &debug_config = DebugConfig());
    ~RuneModel() noexcept;

    // 禁止复制构造和赋值，允许移动构造和移动赋值
    RuneModel(const RuneModel &) = delete;
    RuneModel &operator=(const RuneModel &) = delete;
    RuneModel(RuneModel &&) noexcept;
    RuneModel &operator=(RuneModel &&) noexcept;

    // 执行神符推理；无有效结果时返回空 vector，模型任务不匹配会在构造时抛出异常
    std::vector<NetRuneResult> netProcess(const cv::Mat &input_image);

private:
    // 具体实现由内部的 YOLOModel 完成，RuneModel 仅作为接口类，对外暴露构造函数和rune获取函数
    std::unique_ptr<YOLOModel> m_model;
};

#include "network_deployment_interface.hpp"

#include "network_deployment_lib.hpp"

#include <stdexcept>
#include <utility>

// ============================================================================
// network_deployment_interface.cpp —— 接口层实现
//
// 这个文件本身几乎没有逻辑，全是"转发 + 一道类型检查"：
//   构造函数 → make_unique<YOLOModel>（真正的脏活都在 lib 里）→ supportsArmor()
//   netProcess → 直接透传给 YOLOModel::netProcess
//
// ★ 唯一值得记住的设计：为什么要有 supportsArmor()/supportsRune() 这道检查？
//   上游把"装甲板模型"和"神符模型"拆成两个类，但底层是同一个 YOLOModel，
//   光看类名分不出模型文件到底是哪种。所以这里在【构造时】就问一次后处理的
//   实际类型（后处理是 auto_detect 按输出形状挑出来的，是唯一可信来源），
//   对不上就立刻抛 —— 早失败，而不是等到 netProcess 才发现拿不到结果
//   对本工程的意义：Detector 构造时如果给错了 engine 文件（比如误传了符模型），
//   会在这里抛 std::invalid_argument，属于"启动就报错"，不会带病跑
// ============================================================================

JsonConfig::JsonConfig(std::string json_path,
                       std::string model_key,
                       std::string model_folder)
    : json_path(std::move(json_path)),
      model_key(std::move(model_key)),
      model_folder(std::move(model_folder))
{}

// 单参数重载：注意它转发到的 YOLOModel(model_path) 里写死了
// async + openvino + GPU + 0.5 + auto_detect，本工程（TensorRT）不要用这个
ArmorModel::ArmorModel(const std::string &model_path)
    : m_model(std::make_unique<YOLOModel>(model_path))
{
    if (!m_model->supportsArmor())
        throw std::invalid_argument("模型后处理类型不是装甲板，无法构造 ArmorModel");
}

// 本工程 Detector 走的就是这个重载，参数含义见头文件里的详细说明
// 注意这里的 make_unique 一返回，模型就已经加载完、引擎已经建好了；
// 紧接着的 supportsArmor() 是构造期唯一的"防呆检查"
ArmorModel::ArmorModel(const std::string &model_path,
                       const std::string &infer_mode,
                       const std::string &deploy_way,
                       const std::string &device,
                       float confidence_threshold,
                       const std::string &postprocess_mode,
                       const DebugConfig &debug_config)
    : m_model(std::make_unique<YOLOModel>(model_path,
                                         infer_mode,
                                         deploy_way,
                                         device,
                                         confidence_threshold,
                                         postprocess_mode,
                                         debug_config))
{
    if (!m_model->supportsArmor())
        throw std::invalid_argument("模型后处理类型不是装甲板，无法构造 ArmorModel");
}

ArmorModel::ArmorModel(const JsonConfig &json_config,
                       const DebugConfig &debug_config)
    : m_model(std::make_unique<YOLOModel>(json_config, debug_config))
{
    if (!m_model->supportsArmor())
        throw std::invalid_argument("模型后处理类型不是装甲板，无法构造 ArmorModel");
}

ArmorModel::~ArmorModel() noexcept = default;
ArmorModel::ArmorModel(ArmorModel &&) noexcept = default;
ArmorModel &ArmorModel::operator=(ArmorModel &&) noexcept = default;

// 纯转发。my_color 的语义和三处过滤规则见头文件里的说明；
// 本工程固定传 2（保留红蓝双方）
// 空指针检查只在移动之后的对象上才有意义（移动走的对象 m_model 为空），
// 正常情况下不会命中
std::vector<NetArmorResult> ArmorModel::netProcess(const cv::Mat &input_image, const int &my_color)
{
    if (!m_model)
        return {};
    return m_model->netProcess(input_image, my_color);
}

// ==================== 神符接口（本工程不使用）====================
// 下面全部是 RuneModel 的转发实现，和上面的 ArmorModel 一一对称，
// 区别只在检查函数换成了 supportsRune()、netProcess 少一个 my_color 参数
// tdt-learning4.4 不实例化 RuneModel，保留原样以便和 26_NNDeployment 上游 diff
RuneModel::RuneModel(const std::string &model_path)
    : m_model(std::make_unique<YOLOModel>(model_path))
{
    if (!m_model->supportsRune())
        throw std::invalid_argument("模型后处理类型不是神符，无法构造 RuneModel");
}

RuneModel::RuneModel(const std::string &model_path,
                     const std::string &infer_mode,
                     const std::string &deploy_way,
                     const std::string &device,
                     float confidence_threshold,
                     const std::string &postprocess_mode,
                     const DebugConfig &debug_config)
    : m_model(std::make_unique<YOLOModel>(model_path,
                                         infer_mode,
                                         deploy_way,
                                         device,
                                         confidence_threshold,
                                         postprocess_mode,
                                         debug_config))
{
    if (!m_model->supportsRune())
        throw std::invalid_argument("模型后处理类型不是神符，无法构造 RuneModel");
}

RuneModel::RuneModel(const JsonConfig &json_config,
                     const DebugConfig &debug_config)
    : m_model(std::make_unique<YOLOModel>(json_config, debug_config))
{
    if (!m_model->supportsRune())
        throw std::invalid_argument("模型后处理类型不是神符，无法构造 RuneModel");
}

RuneModel::~RuneModel() noexcept = default;
RuneModel::RuneModel(RuneModel &&) noexcept = default;
RuneModel &RuneModel::operator=(RuneModel &&) noexcept = default;

std::vector<NetRuneResult> RuneModel::netProcess(const cv::Mat &input_image)
{
    if (!m_model)
        return {};
    return m_model->netProcess(input_image);
}

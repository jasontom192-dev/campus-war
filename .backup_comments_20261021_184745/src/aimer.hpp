#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "types.hpp"
#include "target.hpp"
#include "solver.hpp"

// ============================================================================
// Aimer —— 自瞄解算器（"打哪块板" + "往哪打"）
//
// 上游 Tracker 已经用 EKF 把车的位置/速度/朝向/转速都估出来了，本类要回答
// 的就只剩两个问题，正好对应题目 4 的两条要求：
//
//   ① 打分选板：画面里 4 块装甲板，这一发该打哪一块？（且尽量别频繁换）
//   ② 提前量解算：子弹飞过去要 t 秒，这段时间车还在动，瞄"现在的位置"必然
//      打空。要算出【子弹到达那一刻装甲板会在哪】，再把它重投影到图像上。
//
// 数据流：
//   main.cpp / auto_aim_node.cpp ──> best_aim() ──> AimResult
//                                  └─> visualizer.cpp 画黄色准星
//   （只在 tracker.has_lock() 且 !diverged() 时才调用）
//
// 坐标系（★ 全项目的约定，别搞混）：
//   - Target 的状态 x 在 pseudo 系（右/前/上），见 target.hpp 顶部那张表。
//   - 本类内部的 plates / aim_3d 也全在 pseudo 系。
//   - 只有 aim_2d 是像素坐标，由 pseudo_to_pixel() 转成相机系后交给
//     Solver::project_world() 投影得到。
//   - pitch 是相对水平面的【发射仰角】绝对值，与相机姿态无关。
//     （题目前提 2：枪口与相机原点重合，所以可以直接当发射角用）
//
// 选板策略（当前）：
//   可见板：|板法线角 − 车心方位角| < VISIBLE_LIMIT(75°)
//   低速档：锁定板仍可见就继续；否则投影到像素，选离画面中心最近的
//   高速档：车在“命中时刻”的预测状态上，选 |δ| 最小的板；
//           锁定板没有被别的板好出 CAR_SWITCH_DEG 就不换
//   前哨站：保持原 argmin/argmax 规则（只改车）
//   车的初始选板和迭代中的重选都用未来状态；前哨站用当前状态
//
// 提前量：不动点迭代——先猜飞行时间 t，外推到 t 时刻的板位置，
//         解弹道得到新的 t，反复直到相邻两轮差 < 1ms。
// ============================================================================

class Aimer {
public:
  // 一次解算的全部产出。
  //   valid / aim_2d         → 可视化画准星
  //   pitch / converged      → 节点发角度 / 开火门
  //   plate_id / flight_time → 可视化选中板 / 离线自检与调试
  struct AimResult {
    bool valid = false;          // 解算成功才为 true，下游必须先判这个
    // 飞行时间不动点迭代【收敛了】（相邻两轮差 < 1ms）。
    // 没收敛时 valid 仍可能是 true（返回最后一次迭代值）——瞄准照发，
    // 但【开火】要等收敛：节点 decide_fire 用它当一道门。
    // best_aim_direct（基地，解析解）恒为 true。
    bool converged = false;
    int  plate_id = 0;           // 选中的是第几块板（0..armor_num_-1）
    Eigen::Vector3d aim_3d;      // 【pseudo 系】子弹到达时刻的瞄准点（已做重力补偿）
    cv::Point2f     aim_2d;      // 上面那点的像素坐标，画准星用
    double flight_time = 0;      // 弹丸飞行时间 t（s），迭代收敛值
    double pitch = 0;            // 发射仰角（rad），世界系
  };

  // bullet_speed : 弹丸初速（m/s）；main.cpp 传 25.0，节点每帧更新
  // proc_delay   : 从“本帧图像曝光”到“子弹出膛”的固定延迟（s），默认 5ms
  Aimer(double bullet_speed = 25.0, double proc_delay = 0.005);

  // 主入口：对锁定的目标做一次完整解算
  //   target     : tracker 里那个正在跑的 Target（只读，全程不改动它）
  //   img_center : 画面中心像素，低速选板策略拿它当"离得近不近"的度量
  //   solver     : 只用来做投影（pseudo → 像素）
  // 返回 valid=false 表示这一帧解不出来（板全不可见 / 弹道无解），调用方跳过
  AimResult best_aim(const Target  & target,
                     const cv::Point2f & img_center,
                     const Solver & solver);

  // 【基地专用】直接瞄准：不预测、不用 EKF，直接对"这一帧观测到的板中心"
  // 做弹道解算 + 重力补偿。
  //   基地是静止结构（板不动），进 EKF 只会添乱（同一块基地的上下两块板还会
  //   映射到同一个 track 互相拽），所以它整条链路都绕开跟踪器。
  //   xyz_state: 板中心在【状态系】下的坐标（右/前/上），节点里就是世界系
  AimResult best_aim_direct(const Eigen::Vector3d & xyz_state, const Solver & solver) const;

  // 清掉“锁定同一块板”的迟滞状态，下一帧重新选板。
  // Tracker 换锁时由 main.cpp / 节点调用；best_aim 内还有第二道保险。
  void reset_lock();

  // ROS 节点用：游戏在 AutoAimState.projectile_speed 里给当前实际初速
  //（规则附录 D：23.2—23.8 m/s），每帧可能不一样，所以要能改。
  void set_bullet_speed(double v) { bullet_speed_ = v; }

private:
  double bullet_speed_;                      // 弹丸初速（m/s）
  double proc_delay_;                        // 固定处理延迟（s）
  static constexpr double g_    = 9.7833;    // 重力加速度（m/s²）
                                             // ⚠ visualizer.cpp 里硬编码的是 9.81，
                                             //   同一物理量两处不一致（影响很小）
  static constexpr double LOW_W = 1.5;       // |omega| 低于它算"没转"（rad/s），
                                             // 是低速/小陀螺两套选板策略的分界
  // 车的高速档换板门槛（照 auto_aim_rps26 的 switch_threshold）：
  //   ① 挑战者要好出锁定板 CAR_SWITCH_DEG 才允许顶掉它（迟滞）；
  //   ② 评估时刻再提前 CAR_SWITCH_DEG/|ω|（上限 0.2s），即“提前选板”。
  //   调大 = 更晚换板。
  static constexpr double CAR_SWITCH_DEG     = 7.5;   // 度
  static constexpr double CAR_SWITCH_MAX_ADV = 0.2;   // 秒
  static constexpr int    MAX_ITER = 10;     // 飞行时间不动点迭代的次数上限
  static constexpr double CONVERGE_EPS = 0.001;  // 收敛判据：两轮飞行时间差 < 1ms
  static constexpr double VISIBLE_LIMIT = 75.0 * CV_PI / 180.0;
                                             // 板的法线与"相机→车中心"方向的最大
                                             // 偏角（±75°，即 150° 的张角锥）

  // ---- 选板的迟滞状态 ----
  // 低速档：锁定的板还可见就不换；高速档：别的板要好出 CAR_SWITCH_DEG 才换。
  int  locked_plate_id_   = -1;    // 已锁定的板编号，-1 表示还没锁
  int  locked_target_name_ = -1;   // 这个锁是锁在哪辆车上（ArmorName 的整数值）

  // 挑出"法线大致朝着相机"的板，返回它们的【下标】数组（不是板本身）
  std::vector<int> visible_plates(const std::vector<Eigen::Vector4d> & plates,
                                  double center_yaw) const;

  // 给定状态算该目标所有板的位姿 [x,y,z,yaw]（和 Target::armor_xyza_list
  // 同款，但作用在任意外推状态上）—— 选板要用未来状态就得能"在 x 上算板"。
  static std::vector<Eigen::Vector4d> plates_of(const Target & t, const Eigen::VectorXd & x);

  // 在给定状态上选板：锁定板在该状态里还可见 → 保持（迟滞）；
  // 否则按 |ω| 走低速/高速两套策略。返回 -1 = 该状态上没有可打的板。
  // 车的调用方传未来状态（到达时刻），前哨站传当前状态。
  int select_plate(const Target & target, const Eigen::VectorXd & x,
                   const cv::Point2f & img_center, const Solver & solver,
                   int locked_id) const;

  // 低速/静止目标的选板：投影到像素，挑离画面中心最近的那块。
  // 前置条件：visible_ids 非空（best_aim 里已经判过 vis.empty()）。
  // x 是“算板位置用的状态”——车传未来状态，前哨站传当前状态。
  int choose_low_speed(const Target & target,
                       const Eigen::VectorXd & x,
                       const std::vector<int> & visible_ids,
                       const cv::Point2f & img_center,
                       const Solver & solver) const;

  // 小陀螺（|omega| >= LOW_W）的选板。
  // 车：评估时刻取“命中 + CAR_SWITCH_DEG/|ω| 提前量”，挑该时刻 |δ| 最小的板；
  //     锁定板没被好出 CAR_SWITCH_DEG 就不换。locked_id = 上一帧选中的板。
  // outpost=true 时走原 argmin/argmax 规则（只改车）。
  int choose_high_speed(const std::vector<Eigen::Vector4d> & plates,
                        const std::vector<int> & visible_ids,
                        double omega, double center_yaw, bool outpost,
                        int locked_id) const;

  // 弹道解算的产出
  struct TrajResult {
    bool unsolvable = false;   // 这个初速够不着（判别式 < 0）
    double fly_time = 0;       // 飞行时间（s）
    double pitch = 0;          // 发射仰角（rad）
  };

  // 斜抛解算：已知初速 v0、水平距离 dist、高度差 h，求仰角和飞行时间。
  // 二次方程有两个根（低伸弹道 / 高抛弹道），取飞行时间短的那个。
  TrajResult solve_traj(double v0, double dist, double h) const;

  // 把 EKF 状态按匀速模型线性外推 dt 秒（不碰 target 的 EKF，纯"试算"）
  static Eigen::VectorXd predict_state(const Eigen::VectorXd & x, double dt);

  // pseudo 系 → 像素。等价于 Target::pseudo_to_camera() + Solver::project_world()，
  // 这里把两步并在一个函数里。
  static cv::Point2f pseudo_to_pixel(const Eigen::Vector3d & p,
                                     const Solver & solver);
};

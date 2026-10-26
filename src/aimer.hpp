#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "types.hpp"
#include "target.hpp"
#include "solver.hpp"

//Aimer：选板和弹道解算
//状态、plate、aim_3d 均在 pseudo 系（右/前/上），aim_2d 为像素
//pitch 是相对水平面的发射仰角，枪口与相机原点重合

class Aimer {
public:
  //一次解算结果
  struct AimResult {
    bool valid = false; //解算成功才为 true
    //飞行时间迭代收敛（两轮差<1ms）；未收敛时 valid 仍可为 true，但不应开火
    bool converged = false;
    int  plate_id = 0; //选中板编号
    Eigen::Vector3d aim_3d; //子弹到达时刻的瞄准点，pseudo 系，已加重力补偿
    cv::Point2f     aim_2d; //瞄准点的像素坐标
    double flight_time = 0; //飞行时间，秒
    double pitch = 0; //发射仰角，弧度
  };

  //bullet_speed 为弹丸初速 m/s，proc_delay 为固定处理延迟 s
  Aimer(double bullet_speed = 25.0, double proc_delay = 0.01);

  //解算锁定目标；目标只读，img_center 为画面中心，失败时 valid=false
  AimResult best_aim(const Target  & target,
                     const cv::Point2f & img_center,
                     const Solver & solver);

  //基地直瞄：不做预测，直接对状态系下的板中心解算弹道
  AimResult best_aim_direct(const Eigen::Vector3d & xyz_state, const Solver & solver) const;

  //清除选板迟滞状态
  void reset_lock();

  //设置弹丸初速，比赛时每帧可能变化（23.2~23.8 m/s）
  void set_bullet_speed(double v) { bullet_speed_ = v; }

private:
  double bullet_speed_; //弹丸初速 m/s
  double proc_delay_; //固定处理延迟 s
  static constexpr double g_    = 9.7833; //重力加速度 m/s²；visualizer.cpp 另用 9.81
  static constexpr double LOW_W = 1.5; //|omega| 低于此值走低速选板，rad/s
  //高速换板迟滞：挑战板需好出 7.5°；评估时刻提前 7.5°/|ω|，最多 0.2s
  static constexpr double CAR_SWITCH_DEG     = 7.5; //度
  static constexpr double CAR_SWITCH_MAX_ADV = 0.2; //秒
  static constexpr int    MAX_ITER = 15; //飞行时间迭代上限
  static constexpr double CONVERGE_EPS = 0.001; //两轮飞行时间差小于 1ms 视为收敛
  //板法线与相机→车中心方向的最大偏角，±75°
  static constexpr double VISIBLE_LIMIT = 75.0 * CV_PI / 180.0;

  //选板迟滞状态
  int  locked_plate_id_   = -1; //已锁定板编号，-1 表示未锁
  int  locked_target_name_ = -1; //锁定目标的 ArmorName 整数值

  //返回法线朝相机的板下标
  std::vector<int> visible_plates(const std::vector<Eigen::Vector4d> & plates,
                                  double center_yaw) const;

  //按给定外推状态计算全部板的 [x,y,z,yaw]
  static std::vector<Eigen::Vector4d> plates_of(const Target & t, const Eigen::VectorXd & x);

  //在给定状态上选板；无可见板时返回 -1
  int select_plate(const Target & target, const Eigen::VectorXd & x,
                   const cv::Point2f & img_center, const Solver & solver,
                   int locked_id) const;

  //低速选板：投影到像素后选离画面中心最近的板；visible_ids 必须非空
  int choose_low_speed(const Target & target,
                       const Eigen::VectorXd & x,
                       const std::vector<int> & visible_ids,
                       const cv::Point2f & img_center,
                       const Solver & solver) const;

  //高速选板；前哨站保持 argmin/argmax，车按提前评估时刻选板
  int choose_high_speed(const std::vector<Eigen::Vector4d> & plates,
                        const std::vector<int> & visible_ids,
                        double omega, double center_yaw, bool outpost,
                        int locked_id) const;

  //弹道解算结果
  struct TrajResult {
    bool unsolvable = false; //判别式小于 0，初速无法命中
    double fly_time = 0; //飞行时间 s
    double pitch = 0; //发射仰角 rad
  };

  //斜抛解算，返回飞行时间较短的解；v0 为初速，dist 为水平距离，h 为高度差
  TrajResult solve_traj(double v0, double dist, double h) const;

  //按匀速模型外推状态 dt 秒，不修改原 EKF
  static Eigen::VectorXd predict_state(const Eigen::VectorXd & x, double dt);

  //pseudo 系转像素
  static cv::Point2f pseudo_to_pixel(const Eigen::Vector3d & p,
                                     const Solver & solver);
};

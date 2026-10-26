#pragma once
#include <opencv2/opencv.hpp>
#include <list>
#include <map>
#include "types.hpp"
#include "tracker.hpp"
#include "solver.hpp"
#include "aimer.hpp"

class Visualizer {
public:
  Visualizer(Solver & solver);

  // draw_prediction: 要不要画"EKF 预测点"那条红箭头。
  //   基地是直瞄（不用 EKF、不预测），箭头画出来只是跟踪器的猜测、跟实际
  //   瞄点不一致，所以节点在直瞄基地时传 false，只留黄准星。
  void draw(cv::Mat & img, const std::list<Armor> & armors,
            const Tracker & tracker, int frame,
            const Aimer::AimResult * aim_result = nullptr,
            bool draw_prediction = true);

private:
  Solver & solver_;

  void draw_box(cv::Mat & img, const Box3D & b3, const cv::Vec3d & rvec,
                const cv::Vec3d & tvec, const cv::Scalar & color, int thickness = 2);
};

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

  //draw_prediction 为 false 时不画 EKF 预测箭头
  void draw(cv::Mat & img, const std::list<Armor> & armors,
            const Tracker & tracker, int frame,
            const Aimer::AimResult * aim_result = nullptr,
            bool draw_prediction = true);

private:
  Solver & solver_;

  void draw_box(cv::Mat & img, const Box3D & b3, const cv::Vec3d & rvec,
                const cv::Vec3d & tvec, const cv::Scalar & color, int thickness = 2);
};

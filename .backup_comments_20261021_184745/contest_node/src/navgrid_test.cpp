// navgrid_test —— 离线对拍工具：C++ NavGrid 规划任务表每段，打印长度。
// 用法：navgrid_test [nav_grid.bin] [mission_stages.json]
// 和 Python 的数字逐段对比（mission_node.py --dry）验证移植口径一致。
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "nav_grid.hpp"

int main(int argc, char ** argv) {
  const std::string bin = argc > 1 ? argv[1]
      : "/home/king/tdt-learning4.5/nav_data/nav_grid.bin";
  const std::string stages_path = argc > 2 ? argv[2]
      : "/home/king/tdt-learning4.5/nav_data/mission_stages.json";
  NavGrid g;
  if (!g.load(bin)) {
    std::printf("栅格读不了: %s\n", bin.c_str());
    return 1;
  }
  std::printf("栅格 %dx%d  res=%.2f  墙折线 %zu 段\n", g.ni, g.nj, g.res,
              g.wall_segs_.size() / 4);

  std::vector<std::pair<double, double>> pts;
  try {
    YAML::Node d = YAML::LoadFile(stages_path);
    for (const auto & s : d["stages"])
      pts.push_back({s["goal"][0].as<double>(), s["goal"][1].as<double>()});
  } catch (const std::exception & e) {
    std::printf("站点表读不了: %s\n", e.what());
    return 1;
  }

  double tot = 0.0;
  for (size_t k = 0; k + 1 < pts.size(); ++k) {
    auto p = g.plan(pts[k].first, pts[k].second, pts[k + 1].first, pts[k + 1].second, 0.4);
    if (!p) {
      std::printf("  第%zu段 (%+.1f,%+.1f)→(%+.1f,%+.1f): 无路 !! \n", k + 1,
                  pts[k].first, pts[k].second, pts[k + 1].first, pts[k + 1].second);
      continue;
    }
    double L = 0;
    for (size_t i = 1; i < p->size(); ++i)
      L += std::hypot((*p)[i].first - (*p)[i - 1].first,
                      (*p)[i].second - (*p)[i - 1].second);
    tot += L;
    std::printf("  第%zu段 (%+7.2f,%+7.2f)→(%+7.2f,%+7.2f)  %5.1f 单位 = %4.1f m  (%zu 点)\n",
                k + 1, pts[k].first, pts[k].second, pts[k + 1].first, pts[k + 1].second,
                L, L / 3.0, p->size());
    if (argc > 3 && std::string(argv[3]) == "-v") {
      for (const auto & q : *p) std::printf("        (%+7.2f, %+7.2f)\n", q.first, q.second);
    }
  }
  std::printf("全程 %.0f 单位 = %.0f m\n", tot, tot / 3.0);
  return 0;
}

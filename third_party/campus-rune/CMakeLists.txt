# ============================================================================
# campus-rune —— RP-26Rune（深圳大学 RobotPilots 2026 能量机关算法）的移植构建
#
# 来源：/home/king/下载/RP-26Rune-main/src/core
#   power_rune/       算法主体（观测精修/Chamfer 位姿/LM-IRLS 大符/联合弹道/火控）
#   transform_tools/  坐标变换（TF 树，依赖 Sophus）
#   time/             时间戳（timetool::Timestamp）
#   function/         时间换算（原依赖 foxglove::Time —— 已用 inc/Time_generated.h 顶替）
#   config_loader/    配置读取（ReJson：OpenCV FileStorage 吃 JSON）
#
# 移植期的三处替换件（都在本目录，原版没动）：
#   inc/Time_generated.h           foxglove::Time 的最小实现（丢 flatbuffers）
#   inc/img_viz.hpp                ImgViz 空实现（原为调试窗口）
#   foxglove_viz/inc/...           foxglove 可视化推送层空实现（SDK 产物没开源）
#
# 依赖：OpenCV / Eigen / Ceres / glog / gflags / Sophus（本机均已装好）
# ============================================================================
file(GLOB_RECURSE CAMPUS_RUNE_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/power_rune/interface/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/power_rune/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/transform_tools/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/function/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/config_loader/src/*.cpp
)

add_library(campus-rune STATIC ${CAMPUS_RUNE_SRC})

target_include_directories(campus-rune PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/foxglove_viz/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/power_rune/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/power_rune/include
    ${CMAKE_CURRENT_SOURCE_DIR}/power_rune/config
    ${CMAKE_CURRENT_SOURCE_DIR}/transform_tools/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/time/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/function/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/config_loader/inc
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

# POSITION_INDEPENDENT_CODE：ROS 节点要把它链进可执行文件，静态库必须带 -fPIC
set_target_properties(campus-rune PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_link_libraries(campus-rune PUBLIC
    ${OpenCV_LIBS}
    ceres
    glog
    gflags
    Sophus::Sophus
)

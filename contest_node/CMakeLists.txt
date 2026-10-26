# ============================================================================
# contest_node/CMakeLists.txt —— 比赛用 ROS 节点 auto_aim_node
#
# ★ 它【不重新实现算法】：本包只提供 src/auto_aim_node.cpp（ROS 接入层：
#   话题订阅/规则校验/状态机/开火决策），算法主体仍取自 ../../src/*.cpp
#   所以改了 src/ 下的文件，这个包和根目录的 auto_aim 都要重新编译
# ★ 本包在 tdt_contest_ws/src 下是软链，源码会被 colcon 拷进 build/ 再编，
#   于是"../../src"在拷贝后就找不到了 —— 所以下面用 REAL_PATH 解析真实路径
# ============================================================================

cmake_minimum_required(VERSION 3.16)
project(auto_aim_contest)

# NNdeployment 部署库要求 C++20（和根 CMakeLists.txt 一致）
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wno-deprecated-enum-enum-conversion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
# RP-26Rune 打符移植件需要（Ceres/glog/gflags 系统装了；Sophus 装在 /usr/local）
find_package(Sophus REQUIRED)

# tdt_interface 由 campusgame_ros 里的源码编译得到：
#   cd campusgame_ros/ros2_auto_framework && ./build.sh
find_package(tdt_interface REQUIRED)

# 本包在 tdt_contest_ws/src 下是符号链接，colcon 会把源码拷到 build 目录再编译，
# 所以要 REALPATH 解析出真实目录，才能找到同级的 ../src
file(REAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" PKG_DIR)
set(AIM_ROOT "${PKG_DIR}/..")
set(AIM_SRC  "${AIM_ROOT}/src")
if(NOT EXISTS "${AIM_SRC}/detector.cpp")
  message(FATAL_ERROR "找不到自瞄算法源码: ${AIM_SRC}")
endif()

# ---------------------------------------------------------------------------
# TensorRT / CUDA 运行时探测 —— 与根 CMakeLists.txt 同一套逻辑：
# 先用 pip 的 tensorrt 包，没有就用 ~/TensorRT（本机是后者）
# ---------------------------------------------------------------------------
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c
          "import os,tensorrt;print(os.path.dirname(os.path.abspath(tensorrt.__file__)))"
  OUTPUT_VARIABLE TRT_PY_ROOT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(TRT_PY_ROOT)
  get_filename_component(SITE_PACKAGES "${TRT_PY_ROOT}" DIRECTORY)
else()
  set(SITE_PACKAGES "/nonexistent-site-packages")
endif()

function(first_existing out_var)
  foreach(pattern IN LISTS ARGN)
    file(GLOB hit "${pattern}")
    if(hit)
      list(GET hit 0 found)
      set(${out_var} "${found}" PARENT_SCOPE)
      return()
    endif()
  endforeach()
  set(${out_var} "" PARENT_SCOPE)
endfunction()

first_existing(_trt_hdr
  "${TRT_PY_ROOT}/include/NvInfer.h"
  "${SITE_PACKAGES}/tensorrt/include/NvInfer.h"
  "$ENV{HOME}/TensorRT*/include/NvInfer.h"
  "/usr/include/x86_64-linux-gnu/NvInfer.h")
first_existing(TENSORRT_LIBRARY
  "${SITE_PACKAGES}/tensorrt_libs/libnvinfer.so"
  "${TRT_PY_ROOT}/libs/libnvinfer.so"
  "$ENV{HOME}/TensorRT*/lib/libnvinfer.so")
first_existing(_cudart_hdr
  "${SITE_PACKAGES}/nvidia/cuda_runtime/include/cuda_runtime_api.h"
  "$ENV{HOME}/TensorRT/cudart/usr/local/cuda-*/targets/x86_64-linux/include/cuda_runtime_api.h"
  "/usr/local/cuda/include/cuda_runtime_api.h")
first_existing(CUDART_LIBRARY
  "${SITE_PACKAGES}/nvidia/cuda_runtime/lib/libcudart.so"
  "$ENV{HOME}/TensorRT/cudart/usr/local/cuda-*/targets/x86_64-linux/lib/libcudart.so"
  "/usr/local/cuda/lib64/libcudart.so")

if(NOT _trt_hdr OR NOT TENSORRT_LIBRARY OR NOT _cudart_hdr OR NOT CUDART_LIBRARY)
  message(FATAL_ERROR "TensorRT/cudart 没找齐 —— 见根 CMakeLists.txt 的说明")
endif()
get_filename_component(TENSORRT_INCLUDE_DIR "${_trt_hdr}" DIRECTORY)
get_filename_component(CUDART_INCLUDE_DIR "${_cudart_hdr}" DIRECTORY)
get_filename_component(CUDART_LIB_DIR "${CUDART_LIBRARY}" DIRECTORY)
get_filename_component(TENSORRT_LIB_DIR "${TENSORRT_LIBRARY}" DIRECTORY)
message(STATUS "TensorRT: ${TENSORRT_INCLUDE_DIR} / ${TENSORRT_LIB_DIR}")
message(STATUS "cudart:   ${CUDART_INCLUDE_DIR} / ${CUDART_LIB_DIR}")

# 第二个参数指定 NNdeployment 的构建目录 = 本包的 build 目录下
# ★ 必须显式指定：默认它会用源树里的路径，而 colcon 的 build 目录会撞上
#   根工程（离线那份）已经用过的源码目录，CMake 会报 "binary directory
#   already used" —— 这是从 4.4 拷工程时踩过的老坑（拷贝连 build/ 一起拷走）
set(NNDEPLOYMENT_ENABLE_TENSORRT ON CACHE BOOL "" FORCE)
add_subdirectory("${AIM_ROOT}/third_party/NNdeployment"
                 "${CMAKE_CURRENT_BINARY_DIR}/nndeployment")

# RP-26Rune 打符算法（静态库 campus-rune）—— 构建目录必须显式指定，理由同上
add_subdirectory("${AIM_ROOT}/third_party/campus-rune"
                 "${CMAKE_CURRENT_BINARY_DIR}/campus-rune")

# ---------------------------------------------------------------------------
# 节点：本包 src/ 是 ROS 接入层，../src 是自瞄算法主体
# ---------------------------------------------------------------------------
add_executable(auto_aim_node
  src/auto_aim_node.cpp
  ${AIM_SRC}/detector.cpp
  ${AIM_SRC}/solver.cpp
  ${AIM_SRC}/tracker.cpp
  ${AIM_SRC}/target.cpp
  ${AIM_SRC}/aimer.cpp
  ${AIM_SRC}/visualizer.cpp
  # 打符（kRune 模式）：2026-10-04 起算法主体在 third_party/campus-rune（静态库），
  # 节点侧只接 rune_campus.cpp 对接层 + 我们的检测器 + 开符计数头文件
  # 老链路（rune_solver/rune_target/rune_picker/rune_aimer）仍留在顶层 src/，
  # 但只给离线对拍工具 rune_aim 用，节点不再编它们
  ${AIM_SRC}/rune/rune_detector.cpp
  ${AIM_SRC}/rune/rune_campus.cpp
  # 全自动任务：导航栅格 + A*（mission:=true 时启用；工具侧 nav_map.py 的移植）
  src/nav_grid.cpp
)

# ---------------------------------------------------------------------------
# 对拍小工具：不开 ROS，只规划任务表里的每一段，打印长度 —— 和 Python 版逐段对拍
#   ./build/auto_aim_contest/navgrid_test [nav_grid.bin] [mission_stages.json]
# ---------------------------------------------------------------------------
add_executable(navgrid_test
  src/navgrid_test.cpp
  src/nav_grid.cpp
)
target_include_directories(navgrid_test PRIVATE ${AIM_SRC} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(navgrid_test ${OpenCV_LIBS} yaml-cpp)
install(TARGETS navgrid_test DESTINATION lib/${PROJECT_NAME})

target_include_directories(auto_aim_node PRIVATE
  ${AIM_SRC}
  ${AIM_SRC}/rune
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(auto_aim_node
  rclcpp
  sensor_msgs
  std_msgs
  tdt_interface
)

target_link_libraries(auto_aim_node
  ${OpenCV_LIBS}
  yaml-cpp
  NNdeployment_lib
  campus-rune
)

# libnvinfer / libcudart 不在系统路径里，写进 RPATH 免得运行时还要设 LD_LIBRARY_PATH
set_target_properties(auto_aim_node PROPERTIES
  BUILD_RPATH "${TENSORRT_LIB_DIR}:${CUDART_LIB_DIR}"
  INSTALL_RPATH "${TENSORRT_LIB_DIR}:${CUDART_LIB_DIR}"
)

install(TARGETS auto_aim_node DESTINATION lib/${PROJECT_NAME})

# ★ 这一行是 run.sh 能跑起来的关键：run.sh 传的 config_file 指向
#   install/auto_aim_contest/share/auto_aim_contest/config_contest.yaml
#   —— 也就是这里装过去的那份副本，【不是】本目录里的原件
#   改完 config_contest.yaml 必须重新 colcon build（--symlink-install 会软链，
#   但如果之前建的是普通拷贝，就必须重编）才会生效
install(FILES config_contest.yaml DESTINATION share/${PROJECT_NAME})

ament_package()

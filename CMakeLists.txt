# ============================================================================
# 根 CMakeLists.txt —— 离线可执行文件 auto_aim（跑视频/摄像头，不开 ROS）
#
# 产物：build/auto_aim
# 组成：src/ 下 7 个 cpp + third_party/NNdeployment（推理）+ OpenCV/Eigen/yaml-cpp
#
# ★ 比赛用的 ROS 节点是另一份构建（contest_node/CMakeLists.txt），
#   它复用同一批 src/*.cpp，但入口换成 auto_aim_node.cpp —— 改算法时两边都会编
# ★ 这个文件和 contest_node/CMakeLists.txt 里有一段【几乎一样】的 TensorRT
#   探测逻辑。改了一处记得同步另一处（两份都找不到 TRT 就 FATAL_ERROR）
# ============================================================================

cmake_minimum_required(VERSION 3.16)
project(auto_aim_demo)

# NNdeployment 部署库要求 C++20（原工程是 C++17，换库时跟着提）
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Python3 不是为了跑脚本，是为了下面那句 `import tensorrt` 去探 pip 包的位置
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
# RP-26Rune 移植件需要（Ceres/glog/gflags 系统装了；Sophus 装在 /usr/local）
find_package(Sophus REQUIRED)

# OpenCV 4.x 的头文件在 C++20 下会刷一屏 deprecated-enum-enum-conversion 警告
# （是 OpenCV 自己的问题，不是我们的代码），关掉免得淹没真正的报错
add_compile_options(-Wno-deprecated-enum-enum-conversion)

# ============================================================================
# TensorRT + CUDA 运行时
#
# 两条路，按顺序试：
#   ① pip 装的 tensorrt-cu13 + nvidia-cuda-runtime-cu13（头文件和 .so 在 site-packages）
#   ② NVIDIA 中国 CDN 的 tarball 解到 ~/TensorRT（本机现在走的是这条）
# 哪条都不通才报错。下面把两条路的候选路径一起列出来，第一个命中的胜出
# ============================================================================
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c
          "import os,tensorrt;print(os.path.dirname(os.path.abspath(tensorrt.__file__)))"
  OUTPUT_VARIABLE TRT_PY_ROOT OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(TRT_PY_ROOT)
  get_filename_component(SITE_PACKAGES "${TRT_PY_ROOT}" DIRECTORY)
  message(STATUS "tensorrt python 包: ${TRT_PY_ROOT}")
else()
  # 没有 pip 的包：给个不存在的目录，让下面 site-packages 那几条候选自然落空
  set(SITE_PACKAGES "/nonexistent-site-packages")
  message(STATUS "没有 pip 的 tensorrt 包，改用本地 TensorRT 目录（~/TensorRT*）")
endif()

# 下面这个函数：按顺序试若干通配路径，返回第一个命中的
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
if(NOT _trt_hdr)
  message(FATAL_ERROR "找不到 NvInfer.h（pip 的 tensorrt wheel 里没有的话，"
                      "去 github.com/NVIDIA/TensorRT/releases 下 Linux tarball 解到 ~/TensorRT）")
endif()
get_filename_component(TENSORRT_INCLUDE_DIR "${_trt_hdr}" DIRECTORY)
message(STATUS "NvInfer.h:        ${TENSORRT_INCLUDE_DIR}")

first_existing(TENSORRT_LIBRARY
  "${SITE_PACKAGES}/tensorrt_libs/libnvinfer.so"
  "${SITE_PACKAGES}/tensorrt_libs/libnvinfer.so.10"
  "${TRT_PY_ROOT}/libs/libnvinfer.so"
  "$ENV{HOME}/TensorRT*/lib/libnvinfer.so")
if(NOT TENSORRT_LIBRARY)
  message(FATAL_ERROR "找不到 libnvinfer.so")
endif()
get_filename_component(TENSORRT_LIB_DIR "${TENSORRT_LIBRARY}" DIRECTORY)
message(STATUS "libnvinfer.so:    ${TENSORRT_LIBRARY}")

# cudart：pip 的 nvidia-cuda-runtime-cu13 装在 site-packages/nvidia/cuda_runtime/
first_existing(_cudart_hdr
  "${SITE_PACKAGES}/nvidia/cuda_runtime/include/cuda_runtime_api.h"
  "$ENV{HOME}/TensorRT/cudart/usr/local/cuda-*/targets/x86_64-linux/include/cuda_runtime_api.h"
  "/usr/local/cuda/include/cuda_runtime_api.h")
first_existing(CUDART_LIBRARY
  "${SITE_PACKAGES}/nvidia/cuda_runtime/lib/libcudart.so"
  "${SITE_PACKAGES}/nvidia/cuda_runtime/lib/libcudart.so.13"
  "$ENV{HOME}/TensorRT/cudart/usr/local/cuda-*/targets/x86_64-linux/lib/libcudart.so"
  "/usr/local/cuda/lib64/libcudart.so")
if(NOT _cudart_hdr OR NOT CUDART_LIBRARY)
  message(FATAL_ERROR "找不到 cudart —— 先 pip install nvidia-cuda-runtime-cu13")
endif()
get_filename_component(CUDART_INCLUDE_DIR "${_cudart_hdr}" DIRECTORY)
get_filename_component(CUDART_LIB_DIR "${CUDART_LIBRARY}" DIRECTORY)
message(STATUS "cuda_runtime_api.h: ${CUDART_INCLUDE_DIR}")
message(STATUS "libcudart.so:       ${CUDART_LIBRARY}")

# 让 third_party/NNdeployment 走 TensorRT 后端（option() 会沿用已存在的 cache 值）
set(NNDEPLOYMENT_ENABLE_TENSORRT ON CACHE BOOL "Build TensorRT backend support" FORCE)
add_subdirectory(third_party/NNdeployment)

# RP-26Rune 打符算法移植（2026-10-04）：静态库 campus-rune
# 接法：离线对拍工具 rune_campus（本文件）和 ROS 节点 kRune 模式
# （contest_node/CMakeLists.txt）都链它，经 src/rune/rune_campus.cpp 对接层调用
add_subdirectory(third_party/campus-rune)

# ============================================================================
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
# 打符的检测/对接/F 键已集中到 src/rune/；老链路（rune_solver 等）
# 仍用 #include "rune_detector.hpp" 的原写法，靠这条 include 路径解析
include_directories(${PROJECT_SOURCE_DIR}/src/rune)

# 源文件是手工列出来的（不是 GLOB）—— 新增 .cpp 时必须在这里补一行，
# 否则会出现"文件明明写了却链接不到符号"的怪错
add_executable(auto_aim
    src/main.cpp
    src/detector.cpp
    src/solver.cpp
    src/tracker.cpp
    src/visualizer.cpp
    src/target.cpp
    src/aimer.cpp
)

target_link_libraries(auto_aim
    ${OpenCV_LIBS}
    yaml-cpp
    NNdeployment_lib
)

# libnvinfer / libcudart 不在系统库路径里（在 site-packages 下），
# 把它们的目录写进 RPATH，免得运行时还要手动设 LD_LIBRARY_PATH
set_target_properties(auto_aim PROPERTIES
    BUILD_RPATH "${TENSORRT_LIB_DIR}:${CUDART_LIB_DIR}"
    INSTALL_RPATH "${TENSORRT_LIB_DIR}:${CUDART_LIB_DIR}"
)

# ---------------------------------------------------------------------------
# 打符离线验证工具：跑 tdt-campus-test 录的素材（video + meta.csv），
# 输出检测/PnP/重投影统计 —— 用来和 Python 版对照，验移植正确性
# 算法文件是 rune_ 前缀那一组；ROS 节点（contest_node）会把它们一起编进去
# ---------------------------------------------------------------------------
add_executable(rune_aim
    src/rune_main.cpp
    src/rune/rune_detector.cpp
    src/rune_solver.cpp
    src/rune_target.cpp
    src/rune_picker.cpp
)

target_link_libraries(rune_aim
    ${OpenCV_LIBS}
    yaml-cpp
    NNdeployment_lib
)

set_target_properties(rune_aim PROPERTIES
    BUILD_RPATH "${TENSORRT_LIB_DIR}:${CUDART_LIB_DIR}"
    INSTALL_RPATH "${TENSORRT_LIB_DIR}:${CUDART_LIB_DIR}"
)

# ---------------------------------------------------------------------------
# RP-26Rune 打符离线验证（2026-10-04 移植）：同一套素材、同样的打印口径，
# 和老链路 rune_aim 对拍用。算法在 third_party/campus-rune（静态库 campus-rune）
# ---------------------------------------------------------------------------
add_executable(rune_campus
    src/rune_campus_main.cpp
    src/rune/rune_campus.cpp
    src/rune/rune_detector.cpp
)

target_link_libraries(rune_campus
    ${OpenCV_LIBS}
    yaml-cpp
    NNdeployment_lib
    campus-rune
)

set_target_properties(rune_campus PROPERTIES
    BUILD_RPATH "${TENSORRT_LIB_DIR}:${CUDART_LIB_DIR}"
    INSTALL_RPATH "${TENSORRT_LIB_DIR}:${CUDART_LIB_DIR}"
)

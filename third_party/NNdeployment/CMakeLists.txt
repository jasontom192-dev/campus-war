# ============================================================================
# NNdeployment —— 26_NNDeployment 统一部署库的构建脚本（第三方，从上游搬入）
#
# 产出两个 target：
#   NNdeployment_lib      动态库，对外只暴露 interface/ 下的 network_deployment_interface.hpp
#   NNdeployment_mpt_lib  静态库，只有性能统计（network_performance.cpp），
#                         用到 MPT::runOfficialBenchmark 时才需要单独链接
#
# ★ 后端是【二选一】编译，不是叠加：NNDEPLOYMENT_ENABLE_TENSORRT=ON 时
#   关掉 OpenVINO，源文件列表里也只进 tensorrt 那个 .cpp。本工程（根 CMakeLists）
#   把它设成 ON，所以本工程只编 TensorRT 后端，不依赖 OpenVINO
#   另外 NNDEPLOYMENT_WITH_TENSORRT / WITH_OPENVINO 这两个宏是
#   本库 PRIVATE 的编译定义，外部拿不到，也不要依赖
#
# ★ 本工程相对上游只动过下面两处（其余保持原样，方便和上游 diff）：
#   1) CUDA：上游是 find_package(CUDAToolkit REQUIRED)（要求系统装了 CUDA toolkit），
#      本机没装，改成"先找系统 toolkit，找不到就用上层传进来的 CUDART_*"，
#      cudart 实际来自 pip 的 nvidia-cuda-runtime-cu13；
#   2) TensorRT：去掉写死的 /usr/src/tensorrt 之类路径依赖，改为可通过
#      -D 传入 TENSORRT_INCLUDE_DIR / TENSORRT_LIBRARY
# ============================================================================
cmake_minimum_required(VERSION 3.22.0)
set(lib_name NNdeployment)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(NNDEPLOYMENT_ENABLE_TENSORRT "Build TensorRT backend support" OFF)

find_package(OpenCV REQUIRED)


set(NNDEPLOYMENT_WITH_OPENVINO ON)
set(NNDEPLOYMENT_WITH_TENSORRT OFF)
if(NNDEPLOYMENT_ENABLE_TENSORRT)
    set(NNDEPLOYMENT_WITH_OPENVINO OFF)
    set(NNDEPLOYMENT_WITH_TENSORRT ON)
    # --- CUDA 运行时 -------------------------------------------------------
    # 原版是 find_package(CUDAToolkit REQUIRED)，即要求装了系统级 CUDA toolkit
    # 本机没装，cudart 来自 pip 的 nvidia-cuda-runtime-cu13，
    # 所以改成：先试系统 toolkit，找不到就用上层传进来的
    # CUDART_INCLUDE_DIR / CUDART_LIBRARY（根 CMakeLists 从 site-packages 里定位）
    find_package(CUDAToolkit QUIET)
    if(NOT CUDAToolkit_FOUND)
        if(NOT CUDART_LIBRARY OR NOT CUDART_INCLUDE_DIR)
            message(FATAL_ERROR "既没找到系统 CUDA toolkit，也没有传入 CUDART_LIBRARY/CUDART_INCLUDE_DIR")
        endif()
        add_library(nndeployment_cudart UNKNOWN IMPORTED)
        set_target_properties(nndeployment_cudart PROPERTIES
            IMPORTED_LOCATION "${CUDART_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${CUDART_INCLUDE_DIR}")
        set(NNDEPLOYMENT_CUDART nndeployment_cudart)
        set(NNDEPLOYMENT_CUDA_INCLUDE "${CUDART_INCLUDE_DIR}")
    else()
        set(NNDEPLOYMENT_CUDART CUDA::cudart)
        set(NNDEPLOYMENT_CUDA_INCLUDE "${CUDAToolkit_INCLUDE_DIRS}")
    endif()

    # TensorRT 没有稳定的官方 CMake package，这里手动查找头文件和库
    # 若上层已经 -D 传了 TENSORRT_INCLUDE_DIR / TENSORRT_LIBRARY，find_* 会直接采用
    find_path(TENSORRT_INCLUDE_DIR NvInfer.h
        PATHS /usr/include/x86_64-linux-gnu
              /usr/src/tensorrt/include
              /usr/local/TensorRT-10.0.1.6/include/
        REQUIRED
    )
    find_library(TENSORRT_LIBRARY nvinfer
        PATHS /usr/lib/x86_64-linux-gnu
              /usr/src/tensorrt/lib
              /usr/local/TensorRT-10.0.1.6/lib/
        REQUIRED
    )
else()
    find_package(OpenVINO REQUIRED)
endif()



set(NNdeployment_src
    ${CMAKE_CURRENT_SOURCE_DIR}/interface/network_deployment_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/network_deployment_lib.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/preprocess/network_preprocess.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/postprocess/network_postprocess.cpp
)

if(NNDEPLOYMENT_WITH_TENSORRT)
    list(APPEND NNdeployment_src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/inference/network_deployment_tensorrt.cpp
    )
else()
    list(APPEND NNdeployment_src
        ${CMAKE_CURRENT_SOURCE_DIR}/src/inference/network_deployment_openvino.cpp
    )
endif()

add_library(${lib_name}_lib SHARED ${NNdeployment_src})

add_library(${lib_name}_mpt_lib STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/performance/network_performance.cpp
)
set_target_properties(${lib_name}_mpt_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${lib_name}_mpt_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/performance
)

# 头文件路径
target_include_directories(${lib_name}_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/interface
    ${OpenCV_INCLUDE_DIRS}
)
target_include_directories(${lib_name}_lib PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/preprocess
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/inference
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/postprocess
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/performance
)

# 链接库
target_link_libraries(${lib_name}_lib PUBLIC
    ${OpenCV_LIBS}
)
target_link_libraries(${lib_name}_lib PRIVATE ${lib_name}_mpt_lib)

if(NNDEPLOYMENT_WITH_OPENVINO)
    target_link_libraries(${lib_name}_lib PRIVATE
        openvino::runtime
    )
endif()

if(NNDEPLOYMENT_WITH_TENSORRT)
    target_include_directories(${lib_name}_lib PRIVATE
        ${NNDEPLOYMENT_CUDA_INCLUDE}
        ${TENSORRT_INCLUDE_DIR}
    )
    target_link_libraries(${lib_name}_lib PRIVATE
        ${TENSORRT_LIBRARY}
        ${NNDEPLOYMENT_CUDART}
    )
    target_compile_definitions(${lib_name}_lib PRIVATE NNDEPLOYMENT_WITH_TENSORRT=1)
else()
    target_compile_definitions(${lib_name}_lib PRIVATE NNDEPLOYMENT_WITH_TENSORRT=0)
endif()

if(NNDEPLOYMENT_WITH_OPENVINO)
    target_compile_definitions(${lib_name}_lib PRIVATE NNDEPLOYMENT_WITH_OPENVINO=1)
else()
    target_compile_definitions(${lib_name}_lib PRIVATE NNDEPLOYMENT_WITH_OPENVINO=0)
endif()

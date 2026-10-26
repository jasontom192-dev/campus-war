# tdt-learning4.5 换机部署包

这个包的目标是：**拷到另一台机器上，装依赖、重编、重编 engine，然后就能跑**。
不是"拷过去双击就能跑"——这个项目依赖 TensorRT/CUDA/ROS，二进制和引擎都不是跨机器通用的。

---

## 0. 包里有什么 / 没有什么

带：

- `src/`、`contest_node/`、`third_party/`（NNdeployment + campus-rune）
- `campusgame_ros/.../tdt_interface` 消息包
- `tdt_contest_ws/` 的 `build.sh` / `run.sh` / `src` 软链（不含 `build/install/log`）
- `models/*.engine`（本机预编，**仅同 GPU + 同 TensorRT 版本可复用**）
- `models/onnx/`（两个 ONNX，目标机重编 engine 用）
- `nav_data/` 的运行时文件（`nav_grid.bin`、`nav_grid.npz`、`mission_stages.json`、`nav_overrides.json`、`hazards.json`、`learned_blocks.json`、地图 PNG）
- `third_party/sophus_install/`（Sophus 头文件 + CMake 配置，免目标机安装）
- `docs/nav-mission-stages.md`（站点编辑稿）
- `deploy/`（本脚本）

不带：

- `build/`、`install/`、`log/`、`.ros/`、`.backup*`
  —— 这些里面全是**本机的绝对路径**，拷过去会让 CMakeCache / colcon 软链指回旧机器。
- `nav_data/manual_*.jsonl`、`nav_probe_*.jsonl`（手工录制轨迹，几十 MB，运行不需要）
- 游戏本体、ROS 桥、TensorRT/CUDA 运行时、NVIDIA 驱动

---

## 1. 目标机最低要求

| 项 | 要求 |
|---|---|
| 系统 | Ubuntu 24.04（推荐；22.04 也能试，但包名/ROS 版本要换） |
| ROS | ROS 2 Jazzy（`/opt/ros/jazzy/setup.bash` 存在） |
| GPU | NVIDIA 独显 + 驱动；**没有 NVIDIA GPU 跑不了 TensorRT** |
| TensorRT | **10.x 完整包**（头文件 + `libnvinfer` + `trtexec` + 目标 GPU 架构的 builder resource） |
| CUDA | CUDA Runtime 13.x（或 pip 的 `nvidia-cuda-runtime-cu13`） |
| OpenCV | 4.6（apt `libopencv-dev`）可试；本机是 4.7.0（`/usr/local`）。4.6 编不过再装 4.7 |
| 其它 | Eigen3、yaml-cpp、Ceres、glog、gflags、fmt、SuiteSparse（`deps` 会装） |
| Python | rclpy、cv2、numpy、PIL（`deps` 会装） |
| 游戏 | tdt-campus-game 能在目标机跑，并在游戏内按 **F8** 启用 ROS 桥 |

> `tdt-campus-game` 和 ROS 桥不在这个包里；包只负责自瞄/导航节点。

---

## 2. TensorRT / CUDA：最容易翻车的一步

### 2.1 engine 不是跨机器的

`models/*.engine` 是打包机上用 **RTX 5060 Laptop（sm120）+ TensorRT 10.16.1 + CUDA 13.2** 编的。
TensorRT plan 文件绑定：

- GPU 架构（sm120 / sm89 / sm86 …）
- TensorRT 版本
- 部分插件/层实现

**换 GPU 或换 TensorRT 版本，必须重编**。直接跑旧 engine 的典型报错：

- `serialization version mismatch`
- `no kernel image is available for execution on the device`
- `Engine deserialization failed`

`deploy.sh check` 会尝试 `trtexec --loadEngine=... --skipInference`；失败就说明要重编。

### 2.2 安装 TensorRT

目标机需要**完整 TensorRT 10.x 包**（不是只有 runtime 的 `libnvinfer`）。
NVIDIA 下载 Linux tarball，解压到 `~/TensorRT`（或者设 `TRT_DIR` 指向它）：

```bash
# 例：TensorRT 10.x Linux x86_64 tar 包
tar -xzf TensorRT-10.x.x.x.Linux.x86_64-gnu.cuda-13.x.tar.gz -C ~
mv ~/TensorRT-10.x.x.x ~/TensorRT
~/TensorRT/bin/trtexec --version
```

CUDA runtime 可以：

- 装 CUDA Toolkit 13.x（`/usr/local/cuda`），或
- `pip install nvidia-cuda-runtime-cu13`（CMake 会在 site-packages 找），或
- 随 TensorRT 包里的 cudart 放到 `~/TensorRT/cudart/...`（本机就是这种）

> 本机的 `~/TensorRT` 里 builder resource 是 `sm120` 的。目标 GPU 不是 sm120 时，
> 这个包可能编不了 engine；请下载目标架构对应的完整 TensorRT 包。
> 只看 `libnvinfer.so` 能加载不够——**builder resource 缺了，trtexec 建 engine 会失败**。

### 2.3 重编 engine

```bash
bash deploy/deploy.sh engines
# 等价于用 trtexec 把 models/onnx/ 下两个 ONNX 各编一遍：
#   models/infantry_v8n.engine
#   models/rune_v8n.engine
```

如果目标机用 pip 的 TensorRT 但没有 `trtexec`，请改用完整 tarball（pip 包不带 `trtexec`）。

---

## 3. 快速步骤

```bash
# 1) 解压到目标机任意目录（路径不要带中文/空格）
tar xzf tdt-learning4.5-deploy-YYYYMMDD-HHMM.tar.gz -C ~
cd ~/tdt-learning4.5

# 2) 装系统/ROS 依赖（需要联网 + sudo；TensorRT/CUDA 见第 2 节）
sudo bash deploy/deploy.sh deps

# 3) 把源码/配置里的 /home/king/tdt-learning4.5 改成当前目录
bash deploy/deploy.sh relocate

# 4) 编译（会生成 tdt_contest_ws/build|install 和 build/）
bash deploy/deploy.sh build

# 5) 检查环境 + 旧 engine 能不能加载
bash deploy/deploy.sh check

# 6) 如果 check 说 engine 加载失败（换 GPU / 换 TRT 必失败）
bash deploy/deploy.sh engines

# 7) 跑全自动任务
bash deploy/deploy.sh run
```

`deploy.sh all` = `relocate → build → engines → check`，适合第一次部署。
如果目标机和打包机 GPU/TRT 完全一致，可以跳过 `engines`，先 `check` 看旧 engine 是否可用。

---

## 4. 路径问题是怎么处理的

这个工程原来把 `/home/king/tdt-learning4.5` 写死在几处源码/配置里。
`deploy.sh relocate` 会扫描包内文本文件，把它们替换成**当前解压目录**：

- `contest_node/config_contest.yaml` → 两个 engine 的路径
- `contest_node/src/auto_aim_node.cpp` → `mission_grid/stages/learned` 默认路径
- `contest_node/src/navgrid_test.cpp`
- `third_party/campus-rune/power_rune/config/json.hpp` → power_rune.json 默认路径
- `contest_node/tools/nav_record.py` → 录制输出目录
- `contest_node/tools/stages_from_md.py` → 站点编辑稿路径
- `tools/build_engine.sh` / `build_engine.py` → ONNX 路径
- `nav_data/mission_stages.json` 里的说明文字
- 文档里的示例路径

`run.sh` / `build.sh` 本身用 `$ws` 相对定位，不需要改。
`CAMPUS_RUNE_CONFIG` 在 `deploy.sh run` 里也会显式设成包内配置，双保险。

> 如果解压后**移动过目录**，请重新跑 `relocate` + `build`（二进制里编进了旧路径）。

---

## 5. 运行前检查

```bash
# 游戏已启动，并在游戏内按 F8 启用 ROS
ros2 topic list | grep -E 'autoaim|target| navigation'
ros2 topic hz /autoaim/frame        # 应该有稳定帧率
```

先**只观察不发指令**（确认检测/跟踪正常）：

```bash
cd tdt_contest_ws
./run.sh
```

再接管云台：

```bash
./run.sh --ros-args -p publish_angles:=true
```

最后全自动（底盘+站点+打符）：

```bash
bash deploy/deploy.sh run
# 等价于 ./run.sh --ros-args -p publish_angles:=true -p auto_fire:=true -p mission:=true
```

> 注意：只有 `publish_angles:=true` 时节点才会发底盘速度（`publish_speed()` 会直接返回），
> 否则只是观察模式。

---

## 6. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| `libnvinfer.so.10: cannot open shared object` | TensorRT 没装/没进 `LD_LIBRARY_PATH`；装完整包或 `export LD_LIBRARY_PATH=$TRT_DIR/lib:$LD_LIBRARY_PATH` |
| `serialization version mismatch` / `no kernel image` | engine 与目标 GPU/TRT 不匹配 → `deploy.sh engines` |
| `trtexec` 建 engine 报 builder resource 缺失 | TensorRT 包只带打包机的 sm120 builder；下载目标 GPU 架构的完整 TensorRT |
| `find_package(OpenCV)` 找到 4.6 后编不过 | 装 4.7 并用 `-DOpenCV_DIR=/path/to/opencv4` 指定；或改代码兼容 4.6 |
| `find_package(Sophus)` 找不到 | 确认 `third_party/sophus_install/share/sophus/cmake` 存在；`deploy.sh build` 会自动设 `CMAKE_PREFIX_PATH` |
| CMake 报 `binary directory already used` | 你拷了旧 `build/`；删掉重新编（本包默认不带） |
| colcon 警告 `tdt_interface is already built in underlay` | 无害；本地包会覆盖。若行为异常，删 `tdt_contest_ws/build|install` 重编 |
| `run.sh` 跑起来但车不动 | 没带 `publish_angles:=true`；或游戏 ROS 桥没开（F8） |
| 检测框全乱/颜色反 | config 的 `enemy_color`、模型/相机；见 `工程结构说明.md` |
| 打符 found 恒 0 | 符灯颜色配置（`rune.target_color`）、二值化阈值 180 — 见 `rp26-rune-swap-into-45` 记录 |

---

## 7. 如果目标机不能联网 / 不能装依赖

两条路：

1. **离线 apt 包**：在同类 Ubuntu 24.04 上 `apt-get download` 所需 deb，带到目标机 `dpkg -i`；
   ROS 包也要一并下载。这个包不包含 apt 缓存。
2. **Docker**：在目标机用 `ros:jazzy` + CUDA/TensorRT 基础镜像，挂载本目录，
   `--network host --gpus all` 运行；engine 仍需按目标 GPU 在容器内重编。
   本包不提供 Dockerfile（目标机 Docker/GPU 支持情况未知）。

如果目标机没有 NVIDIA GPU，这个工程当前配置跑不了；需要改 OpenVINO/CPU 后端（工程里
NNdeployment 有 OpenVINO 后端，但本工程未接）。

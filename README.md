# qianli_basic_train

基于 ROS 2 的装甲板识别与位姿解算 demo。使用 YOLO（ONNX Runtime 推理）检测装甲板，再通过 PnP 解算出目标在机器人坐标系下的坐标。

## 功能包组成

| 功能包 | 说明 |
| --- | --- |
| `armor_detector` | YOLO 装甲板识别 + PnP 位姿解算节点，发布识别结果与调试图像 |
| `image_publisher` | 从指定文件夹循环读取图片并发布到图像话题，用于离线测试 |
| `aim_interfaces` | 自定义消息 `AimInfo`（`int16[] coordinate`、`int16 type`） |

## 依赖

### 系统环境

- **Ubuntu 22.04**
- **ROS 2 Humble**（本文命令基于 `/opt/ros/humble`）
- **colcon**（`colcon-common-extensions`）
- C++17 编译器（GCC/Clang）、CMake ≥ 3.8

### ROS 2 依赖

以下依赖可通过 `apt install ros-humble-<pkg>` 安装（或用 `rosdep` 一次装齐）：

- `rclcpp`
- `sensor_msgs`
- `std_msgs`
- `cv_bridge`
- `image_transport`
- `ament_index_cpp`
- `rosidl_default_generators` / `rosidl_default_runtime`（构建 `aim_interfaces` 消息）
- `rqt_image_view`（查看调试图像，可选）

推荐用 `rosdep` 自动安装：

```bash
cd /mnt/c/Users/ty/Desktop/rm_wk/qianli_basic_train
rosdep install --from-paths src --ignore-src -r -y
```

### 第三方库

- **OpenCV**（组件：`core imgproc dnn calib3d`）—— 通常随 ROS 2 一并安装
- **ONNX Runtime 1.18.1** —— 已随仓库附带在
  `src/armor_detector/third_party/onnxruntime/`（含 `include/` 与 `lib/libonnxruntime.so`），
  由 CMake 以 IMPORTED 目标链接，无需额外安装。

### 模型文件

- YOLO 模型：`src/armor_detector/models/armor_yolo11s.onnx`（随仓库附带）。
- 相关参数见 `src/armor_detector/config/armor_detector.yaml`
  （`input_size`、`confidence_threshold`、相机内参 `camera_matrix`、畸变 `distortion` 等）。

## 编译

```bash
cd /mnt/c/Users/ty/Desktop/rm_wk/qianli_basic_train
source /opt/ros/humble/setup.bash
colcon build --symlink-install --allow-overriding image_publisher
source install/setup.bash
```

## 运行

> 每个终端都需要先 `source /opt/ros/humble/setup.bash` 和本工作区的 `install/setup.bash`。

### 终端 1 —— 启动识别节点

```bash
cd /mnt/c/Users/ty/Desktop/rm_wk/qianli_basic_train
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch armor_detector armor_detector.launch.py
```

### 终端 2 —— 循环发布测试图片

```bash
cd /mnt/c/Users/ty/Desktop/rm_wk/qianli_basic_train
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch image_publisher image_publisher.launch.py
```

### 终端 3 —— 显示识别结果图（带识别框、类别和坐标）

```bash
source /opt/ros/humble/setup.bash
source /mnt/c/Users/ty/Desktop/rm_wk/qianli_basic_train/install/setup.bash
ros2 run rqt_image_view rqt_image_view /aim_debug
```

如果没有自动选中，在窗口顶部选择 `/aim_debug`。

### 终端 4 —— 查看坐标消息

```bash
source /opt/ros/humble/setup.bash
source /mnt/c/Users/ty/Desktop/rm_wk/qianli_basic_train/install/setup.bash
ros2 topic echo /aim_target
```

## 话题与消息格式

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/aim_debug` | `sensor_msgs/Image` | 带识别框、类别与坐标的调试图像 |
| `/aim_target` | `aim_interfaces/AimInfo` | 目标坐标与类别 |

实测 `/aim_target` 消息格式：

```yaml
coordinate:
- 1203
- -91
- -2264
type: 2
```

- `coordinate` 当前单位为**毫米**，顺序为机器人坐标系 `[x, y, z]`。
- `type` 为识别到的目标类别编号。

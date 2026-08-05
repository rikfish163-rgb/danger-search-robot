# 快速启动

本文面向第一次运行比赛仿真环境的选手。除特别说明外，以下命令均在 SimEnv 仓库根目录执行。

## 运行要求

- Ubuntu 20.04 或兼容环境
- ROS Noetic，建议安装 `ros-noetic-desktop-full`
- Gazebo Classic
- Python >= 3.8
- `python3-yaml`
- `numpy`，用于评估脚本
- CUDA >= 11.7
- libtorch C++ 版本，用于 Unitree A1 控制器

libtorch 和 CUDA 路径在 `src/unitree_guide/unitree_guide/unitree_guide/CMakeLists.txt` 中配置。如部署路径不同，需要按实际机器调整。

## 编译

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
```

## 一键启动

```bash
./auto.sh
```

`auto.sh` 会执行以下流程：

1. 清理旧的 Gazebo、roslaunch、`junior_ctrl`、门/电梯控制服务和可选虚拟手柄进程。
2. 生成随机楼栋、危险源、干扰源和真值文件。
3. 写入 `generated_building/competition_scene.world`。
4. 启动 Gazebo、Unitree A1 模型、传感器、状态话题和控制器接口。
5. 启动 `building_generator_classic` 门/电梯控制服务。
6. 启动 `devel/lib/unitree_guide/junior_ctrl`。

Livox 点云插件启动时会读取扫描模式 CSV 文件，启动后前十几秒可能出现 `rostopic hz /scan` 暂时显示 `no new messages`。请等待 `auto.sh` 完成并再等待数秒后检查传感器话题。

## 常用启动方式

固定随机种子，便于复现实验：

```bash
SEED=77 ./auto.sh
```

无 GUI 启动，适合远程服务器或性能较弱机器：

```bash
GUI=false ./auto.sh
```

调大场景规模：

```bash
SEED=20260507 FLOOR_COUNT=4 ROOMS_PER_FLOOR=5 DANGER_COUNT=5 DISTRACTOR_COUNT=8 ./auto.sh
```

不启动控制器，只启动环境：

```bash
START_CONTROLLER=0 ./auto.sh
```

只开启 RealSense 深度相机数据，关闭 Livox 和其他比赛传感器数据：

```bash
ENABLE_SENSOR_DATA=0 ENABLE_REALSENSE=1 ./auto.sh
```

只开启 Livox 雷达数据：

```bash
ENABLE_SENSOR_DATA=0 ENABLE_LIVOX=1 ./auto.sh
```

## 启动参数

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `SEED` | 空 | 场景随机种子。为空时自动生成随机种子并写入 manifest |
| `FLOOR_COUNT` | `3` | 楼层数，支持单值 |
| `ROOMS_PER_FLOOR` | `4` | 每层房间数，支持单值 |
| `BUILDING_WIDTH` | `20.0` | 楼栋宽度，单位 m |
| `BUILDING_LENGTH` | `36.0` | 楼栋长度，单位 m |
| `DANGER_COUNT` | `3:6` | 危险源数量，支持 `min:max` |
| `DISTRACTOR_COUNT` | `4:8` | 干扰源数量，支持 `min:max` |
| `GUI` | `true` | 是否启动 Gazebo GUI |
| `PAUSED` | `false` | Gazebo 启动后是否暂停 |
| `START_CONTROLLER` | `1` | 是否启动 `junior_ctrl` |
| `CONTROLLER_FOREGROUND` | `1` | 是否在前台运行控制器 |
| `START_BUILDING_CONTROL` | `1` | 是否启动楼栋门/电梯控制服务 |
| `ROBOT_SPAWN_TIMEOUT` | `120` | 等待 Gazebo 完成机器人模型生成的最长时间，单位 s |
| `CONTROLLER_SPAWNER_TIMEOUT` | `120` | 等待 Gazebo 暴露 controller_manager 接口的最长时间，单位 s |
| `UNITREE_CTRL_DT` | `0.004` | `junior_ctrl` 控制周期，单位 s |
| `UNITREE_LOG_WAIT_WARNINGS` | `0` | 是否输出 `absoluteWait is not enough` 控制周期超时提示 |
| `ENABLE_SENSOR_DATA` | `1` | 比赛传感器数据默认总开关；具体传感器可用下列变量覆盖 |
| `ENABLE_LIVOX` | 跟随 `ENABLE_SENSOR_DATA` | 是否发布 Livox 雷达 `/scan` |
| `ENABLE_LIVOX_IMU` | 跟随 `ENABLE_LIVOX` | 是否发布 `/livox/imu` |
| `ENABLE_REALSENSE` | 跟随 `ENABLE_SENSOR_DATA` | 是否发布 RealSense RGB、深度图和深度点云 |
| `ENABLE_DEPTH_CAMERA` | 空 | `ENABLE_REALSENSE` 的别名，便于只控制深度相机 |
| `ENABLE_FRONT_CAMERA` | `0` | 是否启用可选前视 RGB 相机 |
| `ENABLE_POINTCLOUD_CONVERTER` | 跟随 `ENABLE_LIVOX` | 是否将 `/scan` 转换为 `/livox/Pointcloud2` 和 `/livox/lidar2` |
| `ENABLE_GROUND_TRUTH` | `1` | 是否发布 Gazebo 真值调试话题 |
| `ENABLE_REFEREE_ODOM` | `1` | 是否发布 `/Odometry_gazebo` 和 `odom -> base` TF |
| `ENABLE_FOOT_CONTACT_SENSOR` | `0` | 是否启用四个足端 ContactSensor 及接触力话题 |
| `START_VIRTUAL_JOY` | `0` | 是否启动虚拟手柄，通常需要 `uinput` 权限 |
| `ROBOT_X` | `0.0` | 机器人出生点 x |
| `ROBOT_Y` | `-3.2` | 机器人出生点 y |
| `ROBOT_Z` | `0.6` | 机器人出生点 z |
| `ROBOT_YAW` | `1.5708` | 机器人出生点 yaw |

性能较弱时建议优先使用：

```bash
GUI=false ./auto.sh
```

如只需要测试感知链路，可暂时不启动控制器：

```bash
START_CONTROLLER=0 ./auto.sh
```

## 单独生成场景

只生成比赛场景，不启动 Gazebo：

```bash
source ./devel/setup.bash
rosrun building_obstacles generate_competition_scene.py \
  --seed 77 \
  --floor-count 3 \
  --rooms-per-floor 4 \
  --width 20 \
  --length 36 \
  --danger-count 4 \
  --distractor-count 6 \
  --output-dir ./generated_building \
  --results-dir ./results
```

## 默认场景规模

默认楼栋尺寸按 Unitree A1 室内探索做了收敛：走廊约 2.2 m，单层默认 4 个房间，建筑占地约 20 m x 36 m。该尺寸保留进门、转向和传感器观测余量，同时避免探索时间主要消耗在长距离行走上。需要提高难度时，可逐步增大 `BUILDING_WIDTH`、`BUILDING_LENGTH` 和 `ROOMS_PER_FLOOR`。

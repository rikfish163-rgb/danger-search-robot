# 比赛仿真环境

本目录为比赛仿真环境，面向 `ROS1 Noetic + Gazebo Classic + Unitree A1`。环境启动时会随机生成多楼层室内楼栋，并同步生成危险源、干扰源、门、电梯、传感器链路和机器人控制接口。

比赛目标是控制机器狗完成未知室内环境探索，识别并输出危险源位置。危险源真值文件仅供裁判评估使用，参赛算法不应读取。

## 选手快速入口

| 你要做什么 | 推荐阅读 |
|------------|----------|
| 第一次启动环境 | [快速启动](docs/quick-start.md) |
| 接入导航、感知或控制算法 | [算法接入接口](docs/algorithm-interfaces.md) |
| 理解楼栋、危险源和干扰源 | [比赛场景规则](docs/competition-rules.md) |
| 控制门和电梯 | [门与电梯控制](docs/doors-and-elevator.md) |
| 输出结果并计算分数 | [结果格式与评估方法](docs/evaluation.md) |
| 查看传感器安装、话题和坐标系 | [传感器与 ROS 话题](docs/sensors-and-topics.md) |
| 处理启动 warning 或服务异常 | [常见问题](docs/troubleshooting.md) |

## 任务描述

- 楼栋为多楼层室内建筑，包含房间、走廊、楼梯、电梯和动态门。
- 危险源为红色球体。
- 干扰源为红色方块和绿色球体。
- 源只生成在房间内部，并避开墙体、家具、其他源和房间门口保留区。
- 参赛算法应输出 `results/detected_danger.json`。
- 公开场景信息写入 `generated_building/team_scene_info.json`。
- 真值文件仅供裁判评估使用，不作为参赛算法输入。

## 启动流程

在 SimEnv 仓库根目录执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j
source ./devel/setup.bash
./auto.sh
```

`auto.sh` 会自动完成随机场景生成、Gazebo 启动、A1 模型与传感器启动、门/电梯控制服务启动和 `junior_ctrl` 控制器启动。更多启动方式见 [快速启动](docs/quick-start.md)。

参赛队伍请统一使用 `./auto.sh` 启动比赛环境。单独生成场景时，也应使用 [快速启动](docs/quick-start.md) 中的 `generate_competition_scene.py` 命令。

常用传感器开关：

```bash
# 关闭所有比赛传感器数据，但保留传感器模型显示
ENABLE_SENSOR_DATA=0 ./auto.sh

# 只开启 RealSense 深度相机/RGB/深度点云
ENABLE_SENSOR_DATA=0 ENABLE_REALSENSE=1 ./auto.sh

# 只开启 Livox 雷达和点云转换
ENABLE_SENSOR_DATA=0 ENABLE_LIVOX=1 ./auto.sh
```

## 算法接口

| 接口 | 类型 | 用途 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 机器人速度指令输入 |
| `/scan` | `sensor_msgs/PointCloud` | Livox Mid-360 原始点云 |
| `/livox/Pointcloud2` | `sensor_msgs/PointCloud2` | 转换后的 Livox 点云 |
| `/trunk_imu` | `sensor_msgs/Imu` | 机体 IMU |
| `/livox/imu` | `sensor_msgs/Imu` | Livox 内置 IMU |
| `/real_sense/rgb/image_raw` | `sensor_msgs/Image` | RealSense RGB 图像 |
| `/real_sense/depth/points` | `sensor_msgs/PointCloud2` | 深度相机点云 |
| `/set_door_state` | service | 门控制 |
| `/call_elevator` | service | 电梯控制 |

`junior_ctrl` 默认以前台方式启动。终端输入 `2` 进入站立状态，输入 `4` 进入 RL 键盘行走模式，`W/S` 前后、`A/D` 左右、`J/L` 转向、空格停止。输入 `6` 切换到 RL `/cmd_vel` 模式，保持原有 `/cmd_vel` 控制逻辑。机器人摔倒后可输入 `8` 复位到出生点，再输入 `2` 重新站立。完整接口见 [算法接入接口](docs/algorithm-interfaces.md)。

## 结果文件

参赛算法完成探索后应生成：

```text
results/detected_danger.json
```

格式：

```json
{
  "exploration_time": 98.76,
  "detected_danger_sources": [
    {"position": [2.34, -1.56, 0.25]}
  ]
}
```

评估命令：

```bash
python3 ./src/building_obstacles/scripts/evaluate_danger.py \
  --truth-file ./results/danger_truth.json \
  --detected-file ./results/detected_danger.json \
  --output-file ./results/evaluation_result.json
```

评分细则和匹配规则见 [结果格式与评估方法](docs/evaluation.md)。

## 关键文件

| 文件 | 说明 | 是否可作为算法输入 |
|------|------|--------------------|
| `generated_building/team_scene_info.json` | 机器人起点、公开门/电梯 ID、允许接口和结果文件路径 | 是 |
| `results/detected_danger.json` | 参赛算法输出文件 | 输出文件 |
| `generated_building/competition_scene.world` | Gazebo 使用的完整比赛世界 | 否 |
| `generated_building/layout_metadata.json` | 楼栋布局、房间、门、电梯和目标点元数据 | 否 |
| `generated_building/door_config.yaml` | 动态门控制配置，由环境服务读取 | 否 |
| `generated_building/elevator_config.yaml` | 电梯控制配置，由环境服务读取 | 否 |
| `generated_building/building_config.json` | 环境内部使用的建筑配置 | 否 |
| `generated_building/scene_manifest.json` | 本次随机场景 manifest | 否 |
| `generated_building/danger_truth.json` | 裁判真值副本，本地调试时可能存在 | 否 |
| `results/danger_truth.json` | 裁判真值文件 | 否 |
| `logs/competition_gazebo.log` | Gazebo/launch 日志 | 否 |
| `logs/building_control.log` | 门/电梯控制服务日志 | 否 |
| `logs/junior_ctrl.log` | 控制器日志 | 否 |

## 参赛注意事项

正式算法应只使用公开接口、传感器话题和 `generated_building/team_scene_info.json`。不要读取真值文件、完整 world 文件或布局元数据作为算法输入。

# 常见问题

本文记录比赛环境启动和运行中常见问题。更多日志可查看 `logs/` 目录。

## `absoluteWait is not enough`

现象：

```text
[WARNING] The waitTime=4000 of function absoluteWait is not enough!
The program has already cost 5110us.
```

含义：`junior_ctrl` 单次控制循环耗时超过目标周期。`waitTime=4000` 表示目标周期为 4000 us，即 250 Hz。该提示不代表场景生成失败。

当前 `auto.sh` 默认设置：

```bash
UNITREE_CTRL_DT=0.004
```

默认情况下，`auto.sh` 会设置 `UNITREE_LOG_WAIT_WARNINGS=0`，不再打印这条刷屏 warning。需要观察控制周期耗时时，可显式开启：

```bash
UNITREE_LOG_WAIT_WARNINGS=1 ./auto.sh
```

仍感觉明显慢动作时，可无 GUI 启动：

```bash
GUI=false ./auto.sh
```

确需进一步降低控制频率时，可显式设置：

```bash
UNITREE_CTRL_DT=0.006 ./auto.sh
```

## `/scan` 一开始显示 `no new messages`

Livox 插件启动时会读取扫描模式 CSV 文件，启动后前十几秒可能暂时没有点云。请等待 `auto.sh` 完成并再等待数秒后检查：

```bash
rostopic info /scan
rostopic hz /scan
rostopic hz /livox/Pointcloud2
```

正常情况下：

- `/scan` 类型为 `sensor_msgs/PointCloud`，publisher 为 `/gazebo`。
- `/livox/Pointcloud2` 类型为 `sensor_msgs/PointCloud2`，publisher 为 `/pointcloud2livox`。
- 频率约 10 Hz。

## 点云频率低或 RViz 看起来卡顿

`rostopic hz` 统计的是整帧点云消息频率，不是雷达点率。当前 Livox 仿真默认约 24000 点/帧、10 Hz，点率约 24 万点/s。

如果看起来明显卡顿，先检查 Gazebo 是否跑满实时：

```bash
gz topic -e /gazebo/performance_metrics
```

若 `real_time_factor` 明显低于 1.0，通常是 CPU/GPU 性能不足或 GUI 负载较高。优先尝试：

```bash
GUI=false ./auto.sh
```

也可以在调试导航或控制时暂时关闭点云转换节点，直接使用 `/scan`：

```bash
ENABLE_POINTCLOUD_CONVERTER=0 ./auto.sh
```

如果只需要某一种传感器，可以关闭默认传感器数据后再单独打开：

```bash
ENABLE_SENSOR_DATA=0 ENABLE_REALSENSE=1 ./auto.sh
ENABLE_SENSOR_DATA=0 ENABLE_LIVOX=1 ./auto.sh
```

足端 ContactSensor 也会带来明显计算负载。若当前任务不需要足端接触力话题，可保持默认关闭；需要使用时再统一开启：

```bash
ENABLE_FOOT_CONTACT_SENSOR=1 ./auto.sh
```

关闭该开关只会停止四个足端接触力话题和对应力箭头的数据来源，不会取消足端碰撞或改变机器人与地面的接触物理。

## `rospack` 提示 too many positional options

现象：

```text
[rospack] Error: failed to parse command-line options: too many positional options have been specified on the command line
```

通常是命令中多写了位置参数，或 shell 没有正确展开 `$(rospack find pkg)`。先确认工作空间已 source：

```bash
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
rospack find a1_description
rospack find unitree_guide
rospack find livox_laser_simulation
```

如果单独执行正常，请检查报错前一条命令或 launch 文件参数。

## Gazebo 没有正常退出或端口被占用

`auto.sh` 启动前会尝试清理旧进程，包括：

- `roslaunch unitree_guide multi_floor_gazeboSim.launch`
- `building_generator_classic_control`
- `gzserver`
- `gzclient`
- `gazebo`
- `junior_ctrl`
- `virtual_joy.py`

如仍异常，可手动检查：

```bash
ps aux | rg "gazebo|gzserver|gzclient|roslaunch|junior_ctrl|building_generator_classic_control"
```

## 门或电梯服务不可用

先确认服务是否存在：

```bash
rosservice list | rg "set_door_state|call_elevator"
```

如服务不存在，可手动启动：

```bash
source ./devel/setup.bash
rosrun building_generator_classic building_generator_classic_control \
  --door-config ./generated_building/door_config.yaml \
  --elevator-config ./generated_building/elevator_config.yaml
```

检查日志：

```bash
tail -n 80 logs/building_control.log
```

## 找不到 ROS 包

如果 `rosrun` 或 `rospack find` 找不到包，通常是没有 source 工作空间：

```bash
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
```

## 评估脚本缺少依赖

评估脚本依赖 `numpy`。如报导入错误，请安装：

```bash
sudo apt install python3-numpy
```

## 虚拟手柄权限问题

`START_VIRTUAL_JOY=1` 会尝试启动虚拟手柄，通常需要 `uinput` 权限。比赛算法一般可以直接发布 `/cmd_vel`，不必开启虚拟手柄。

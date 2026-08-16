# 三层危险源自主搜索工程（catkin_ws0803_python）

Unitree A1 四足机器人 + Gazebo 仿真 + FAST-LIO SLAM + move_base/TEB + Graph NBV 探索 + 视觉危险源检测。

## 系统架构

```
SimEnv (Gazebo, 三层建筑)
  ├── unitree_guide (A1 机器人控制器, junior_ctrl)
  ├── Mid360_imu_sim (Livox MID360 激光 + IMU 仿真)
  └── building_generator_classic (门/电梯控制)
        ↓
FAST-LIO (SLAM, /Odometry, camera_init 系)
  ├── fastlio_level_tf (map_level→camera_init)
  ├── world_map_level_calibrator (world→map_level 标定)
  ├── nearest_azimuth_projection_node (/map_confirmed 2D 地图)
  ├── move_base + TEB (导航, world 系)
  ├── path_collision_forward_recovery (卡死直行恢复)
  ├── graph_nbv_stage_b31 (Graph NBV 探索决策)
  └── vision_stack (YOLO + 危险源定位/结果写入)
```

## 关键配置

| 模块 | 文件 | 说明 |
|---|---|---|
| NBV 探索 | `src/danger_search_robot/exploration/graph_nbv/config/graph_nbv_stage_b31_manual_gate.yaml` | 候选点/门禁/frontier 参数 |
| 导航 | `src/danger_search_robot/navigation/config/*.yaml` | TEB/costmap（TEB map_frame=world） |
| SLAM | `src/FAST_LIO/config/mid360.yaml` | FAST-LIO 参数（IMU 倾斜补偿方案见 results/） |
| 视觉 | `src/quadruped_vision/` | YOLO 危险源检测（权重 best.pt = red_sphere 等 3 类） |

## 启动流程

见 `Unitree_A1_三层危险源自主搜索任务完整操作说明_2026-07-31.md`（13 步启动顺序）：
SimEnv → 站立(按2) → livox bridge → FAST-LIO → level_tf → world标定 → 2D投影 → move_base_teb → vision_stack → 切cmd_vel(按6) → 手动到走廊起点 → graph_nbv。

## 已知问题与修复（2026-08-07）

### 卡死问题（成员B：Graph NBV 探索决策）

| 问题 | 根因 | 修复 |
|---|---|---|
| NBV 选不出目标 (`finite_targets=0`) | 候选点采样过严(离墙0.51m)、图路由过窄 | 候选点放宽到0.35、edge_min_clearance=0.10（三层实测0.15起可达性断崖，不误伤门洞） |
| 障碍物附近掉头卡死 | 目标点可离墙很近，掉头空间不足 | `preferred_clearance 0.55→0.75`（目标偏好离墙≥0.75m） |
| 末端房间 frontier 识别不到 | `global_frontier_min_cells=4` 滤掉被幻影切碎的小组件 | `global_frontier_min_cells 4→2` + small_discarded 诊断日志 |
| 走廊起点停歪/门控失效 | 标定误差，gate 未验证朝向 | gate 初始化增加朝向偏差诊断（>30° 告警） |
| rotate recovery 永久卡死 | 依赖不存在的 `/odom` 话题 | 禁用 rotate recovery + /odom relay |
| TEB 帧不一致 | map_frame=map_level vs costmap world | map_frame 改为 world |
| 角落/障碍旁振荡卡死 | 不会后退、路径贴边、yaw 容差严 | TEB: allow_backwards=true、min_obstacle_dist 0.20、yaw_tol 0.20 |
| SLAM 爆炸/移动跟丢 | 点云 line 全 0 但 scan_line=6、fov=90 视野错、容器跑旧二进制 | mid360: scan_line=1、fov=360、det_range=100、extrinsic_T 修正 + 恢复容器编译二进制 |
| level_tf 断链 | static_transform_publisher 不发布 /tf_static | 用 `static_level_tf.py`（StaticTransformBroadcaster）替代 |

### 运行时诊断

`graph_nbv_stage_b31_manual_gate_node.py` 发布锁存的
`/graph_nbv/runtime_status` JSON 心跳（schema=`graph_nbv_runtime_v1`），包含：

- 当前状态、状态持续时间、地图/TF 新鲜度和最后一次错误；
- 活动目标与上一个目标的结果、耗时、失败原因；
- 门禁是否锁定、前向耗尽计数、解锁次数；
- 目标成功/失败总数、连续失败预算、黑名单规模。

前向区域连续无可达 frontier 达到 `forward_finish_stable_cycles` 后，默认只
解锁全局 relocation，让剩余房间仍有机会被搜索；这一步不直接发布倒车速度。
只有最终状态为 `FINISHED` 且目标/房间记录满足验收条件，才算探索完成。

## 解决方案文档

- 任务文档（v1.2）：`/home/hetaisheng/卡死问题任务文档_2026-08-07.docx`，生成脚本 `results/gen_task_doc.py`
- 诊断报告：`/home/hetaisheng/机器人卡死问题诊断报告_2026-08-06.md`
- 成员B 工作说明：`/home/hetaisheng/成员B_Graph_NBV_候选点gate_frontier筛选_工作说明.docx`

## 固定 ROS1 容器环境

当前工作区使用一个固定的 ROS Noetic 镜像 digest，并且只允许一个专用容器挂载本工作区：

```text
image:   danger-search-robot/ros1-fixed:20260816
base:    osrf/ros@sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797
mount:   <this workspace> -> /root/catkin_ws
deps:    /media/hetaisheng/044A81D94A81C83E/ros1_isolated_local/deps (read-only)
network: ros1-simenv-recovery (private bridge)
ROS:     http://127.0.0.1:11311 inside the container
container: simenv-ros1-recovery
```

宿主机从本工作区执行：

```bash
tools/ros1_fixed_container.sh up
tools/ros1_fixed_container.sh status
tools/ros1_fixed_container.sh exec bash
```

不要同时运行旧的 `simenv-run0810*` 或其他 `simenv-ros1-*` 容器；启动脚本会拒绝这种共享工作区/ROS 状态的情况。进入容器后，先启动 roscore 和 `src/SimEnv/auto.sh`，将 `START_CAMERA_BRIDGES=1` 固定打开，再使用下面的固定启动顺序：

```bash
docker exec -it simenv-ros1-recovery bash
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash
nohup roscore > /tmp/roscore_fixed.log 2>&1 < /dev/null &
START_CAMERA_BRIDGES=1 GUI=false PAUSED=true AUTO_UNPAUSE=1 \
  CONTROLLER_FOREGROUND=0 src/SimEnv/auto.sh > /tmp/ros1_fixed_auto.log 2>&1 &
tools/prepare_exploration_stack.sh
tools/run_three_floor_rerun.sh
```

`prepare_exploration_stack.sh` 会初始化 0 层状态、打开入口和三部电梯门，按依赖顺序启动真值 TF、Gazebo RGB/深度桥接、vision_stack 和 FAST-LIO 二维投影。`run_three_floor_rerun.sh` 启动前还会检查 ROS master、关键节点、点云/RGB/深度/YOLO 发布者和电梯/地图服务；检查不通过就退出，不会把“视觉没启动”的过程误记成探索结果。

探索任务的高频日志先写入容器 `/tmp`。任务进程退出后，`tools/publish_three_floor_runtime.sh` 才在有超时保护的独立步骤中把摘要和主日志发布回 `results/`，避免 NTFS bind mount 的慢 I/O 把 ROS 回调和任务主进程一起卡在 `D` 状态。Gazebo 使用软件渲染时仍可能低于实时速度；这属于性能风险，不应与容器/ROS 主节点串线混为一谈。

## 下次快速复刻（固定基线）

本仓库已经把上一次验收过的运行环境固定为一个可复刻基线：

- Docker 镜像 `danger-search-robot/ros1-fixed:20260816`，ID 为
  `sha256:433e256ca01f0333da51020a4a9d909334ef36ec5720e32e86bf49eded63cd89`；
- 容器 `simenv-ros1-recovery`，私有网络 `ros1-simenv-recovery`，原生构建空间为
  `/dev/shm/ros1_recovery`；
- 三层四房间场景种子 `3632072`，危险源 4 个、干扰物 8 个；场景生成物已锁定在
  `generated_building/`，入口台阶修复也已锁定在生成器 exporter 中；
- 固定运行时包含 RGB/depth bridge，且 YOLO/投影节点的原生二进制 SHA256 也会在启动前检查。

下一次在本机直接执行，不要手工重新拼启动顺序：

```bash
cd "/media/hetaisheng/044A81D94A81C83E/catkin_ws0810 移动不卡死，可以倒车/catkin_ws"
tools/replay_ros1_fixed.sh check
tools/replay_ros1_fixed.sh all
```

`all` 会重启这个固定容器以清掉残留 ROS/Gazebo 进程，按固定种子重新生成场景，初始化三层探索栈，执行完整三层搜索，最后调用官方 Gitee evaluator。上次同一基线的客观验收是 `truth=4 / detected=4 / correct=4 / missed=0 / false_alarms=0 / total=37/37`，探索时间 `357.80s`。

只需要启动而不立即跑任务时使用 `tools/replay_ros1_fixed.sh start`；需要从零重启仿真使用 `tools/replay_ros1_fixed.sh restart`；只重置支持栈使用 `tools/replay_ros1_fixed.sh prepare`。固定参数和校验值集中在 `tools/ros1_fixed_baseline.env`，入口脚本会在任务前拒绝镜像、场景、挂载或原生二进制不一致的状态。

不要直接运行不带 `SEED` 的 `src/SimEnv/auto.sh`，因为它会随机生成另一套建筑和危险源位置；复刻任务统一走 `tools/replay_ros1_fixed.sh`。

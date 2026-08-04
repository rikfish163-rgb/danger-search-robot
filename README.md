# Unitree A1 三层危险源自主搜索系统 (ROS1 Noetic + Gazebo)

基于 [SimEnv](https://gitee.com/guoyulun/SimEnv) 的三层全自主危险源搜索工程。
四足机器人(Unitree A1)在仿真的三层建筑中自主完成 SLAM 建图、路径规划、视觉识别与电梯跨层搜索。

## 系统架构

```
SimEnv(Gazebo, 3层楼+电梯+门)
  → unitree_livox_bridge.py (Livox 数据桥接)
  → FAST-LIO (mapping_mid360, SLAM 建图) → /Odometry, /cloud_registered
  → fastlio_level_tf (map_level → camera_init 水平化)
  → world_map_level_calibration / reanchor (world ↔ map_level 标定)
  → fastlio_2d_projection (3D 点云 → 每层 2D 地图 /map_confirmed)
  → move_base + TEB 导航 (global_frame=world)
  → vision_stack (YOLO → 深度定位 → TargetManager → ResultWriter)
  → Graph NBV 探索 (每层 400s 仿真时间)
  → elevator_floor_transition.py (电梯跨层, floor_state.json 管理)
```

## 关键坐标链

```
world → map_level → camera_init → body
```

RViz Fixed Frame 必须为 `world`。楼层编号:0=一楼,1=二楼,2=三楼。

## 启动顺序 (详见 `Unitree_A1_三层危险源自主搜索任务完整操作说明_2026-07-31.md`)

```text
SimEnv ./auto.sh → 按2站立 → 开全部门(main_entrance+电梯三层) → 初始化楼层状态
→ Livox bridge → FAST-LIO → fastlio_level_tf → world_map_level_calibration
→ fastlio_2d_projection → 首次 clear_map → move_base_teb → vision_stack
→ 按6 → mission_manager → Graph NBV(400s)
```

## 环境要求

- Docker 容器 `osrf/ros:noetic-desktop-full`(Ubuntu 20.04 / ROS Noetic / Gazebo Classic)
- GPU 直通用于 Gazebo 渲染(容器内 nvidia-smi 可见)
- **YOLO 固定 CPU**(RTX 5070 sm_120 与容器 PyTorch 不兼容)
- `catkin_ws` 挂载到容器 `/root/catkin_ws`,SimEnv 独立构建并作 underlay

## 本次修复记录

### 1. SimEnv livox 插件段错误 (livox_points_plugin.cpp)
**根因**:`ros::init(argc, argv, curr_scan_topic)` 使用 `/scan` 作为节点名,
`roscpp` 抛 `InvalidNodeNameException`(节点名不能含 `/`),未捕获异常在 Gazebo
插件加载边界崩成段错误,导致 gzserver 世界加载崩溃。
**修复**:改为合法节点名 `"livox_points_plugin"`。
**文件**:`SimEnv/src/Mid360_imu_sim/src/livox_points_plugin.cpp:76`

### 2. A1 关节 2π 缠绕导致站立/行走失败 (joint_controller.cpp)
**根因**:Gazebo ros_control 报告的关节角对负角做 +2π 缠绕
(如 calf 物理 -2.697 报告为 3.586),UnitreeJointController 读到错误值
计算巨大误差,力矩打满(effort=-33.5)无法站立/行走。
**修复**:控制器 update 中对 `currentPos` 做 2π 解缠绕,将其调整到
URDF 关节限制范围内,使误差计算正确。
**文件**:`SimEnv/src/unitree_guide/unitree_ros/unitree_legged_control/src/joint_controller.cpp`

### 3. Graph NBV 门附近候选点过滤过严 (同学B 任务)
按 `解决门附近不动问题排查与修复建议.docx` 同学B 范围调整:
`graph_nbv_stage_b31_manual_gate.yaml`:
- `min_candidate_clearance: 0.51 → 0.35`
- `preferred_clearance: 0.80 → 0.55`
- `weight_wall_proximity: 5.0 → 1.0`
- `global_approach_min_distance: 0.45 → 0.25`
- `global_approach_max_distance: 1.60 → 1.20`
- `gate_backtrack_margin: 0.30 → 0.45`
- `candidate_spacing: 0.80 → 0.60`
- `blacklist_radius: 1.00 → 0.60`
- `goal_min_distance: 0.70 → 0.40`
- `visibility_wall_dilation: 0.12 → 0.08`

并新增诊断日志(`graph_nbv_stage_b31_manual_gate_node.py`):
- `[nbv] gate blocks candidate (x, y) progress=... margin=...` — 门控拦截候选点时输出
- `[nbv] candidate filter: known_free=N clearance_fail=M min_clear=...` — 候选点被 clearance 过滤统计

### 4. Docker 存储迁移
Docker 数据根从系统盘迁到数据盘 ext4 loop 镜像(`/media/.../ros1simenv_docker.img`,
挂载 `/mnt/ros1docker`),解决系统盘 99% 满无法拉取 osrf/ros 镜像的问题。

## 已知限制

- A1 在部分 SimEnv 版本下关节角度报告存在 ±2π 缠绕,已通过控制器解缠绕缓解;
  若仍出现无法站立,按 `stand_fix.sh` 方式先复位关节到站立构型再按 2。
- 门附近路径规划可能因地图/costmap 对门洞判定过严而 `NO PATH`,
  需结合同学B/C 的参数调整与 RViz 可视化对比。

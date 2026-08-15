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

## 容器环境（simenv-run）

- 镜像：`osrf/ros:noetic-simenv-ready`（重建脚本 `simenv_rebuild.sh` / `simenv_init.sh`）
- 启动：`simenv_restore.sh`（宿主）
- 注意：容器 llvmpipe 软件渲染下仿真 0.36x 非实时，SLAM 长时间运行会发散；GPU 实时环境正常

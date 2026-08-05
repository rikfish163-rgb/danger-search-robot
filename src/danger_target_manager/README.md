# danger_target_manager

本功能包接收单帧三维危险源观测，完成候选关联、连续帧确认、历史去重和JSON结果写入。图像检测、点云处理和坐标变换由上游功能包完成。

## 接口

| 接口 | 默认名称 | 类型 |
|---|---|---|
| 观测输入 | `/danger_observation` | `DangerObservation` |
| 已确认目标输出 | `/confirmed_danger` | `ConfirmedDanger` |
| 目标管理复位 | `/target_manager/reset` | `std_srvs/Trigger` |
| 结果写入复位 | `/danger_result_writer/reset` | `std_srvs/Trigger` |
| 探索计时开始 | `/danger_result_writer/start` | `std_srvs/Trigger` |
| 结果固化 | `/danger_result_writer/finalize` | `std_srvs/Trigger` |

同一RGB帧中的多个目标应使用相同的`header.stamp`连续发布多条`DangerObservation`。`valid=false`表示未命中帧，不会创建新候选目标。

## 启动

```bash
roslaunch danger_target_manager target_manager.launch \
  input_topic:=/danger_observation \
  output_topic:=/confirmed_danger \
  expected_frame:=world
```

JSON写入器由`vision_system_bringup/vision_pipeline.launch`统一启动。完整参数、结果路径优先级和运行顺序见交付包根目录`README.md`。


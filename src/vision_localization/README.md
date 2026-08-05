# vision_localization

本功能包将二维目标检测和与RGB对齐的有组织点云同步，在检测框内提取有效XYZ点，使用0.15 m半径约束的球体RANSAC估计球心，并在RGB采集时刻通过TF2转换到配置的全局坐标系。

## 默认接口

| 方向 | 话题 | 类型 |
|---|---|---|
| 输入 | `/yolo/detections` | `quadruped_vision/DetectionArray` |
| 输入 | `/real_sense/depth/points` | `sensor_msgs/PointCloud2` (`real_sense_optical_frame`) |
| 输入 | `/tf`, `/tf_static` | TF2 |
| 输出 | `/danger_observation` | `danger_target_manager/DangerObservation` |
| 调试输出 | `/vision_localization/candidate_camera` | `geometry_msgs/PointStamped` |
| 调试输出 | `/vision_localization/candidate_world` | `geometry_msgs/PointStamped` |
| 调试输出 | `/vision_localization/roi_points` | `sensor_msgs/PointCloud2` |

点云必须为有组织点云，宽高必须与`DetectionArray.image_width/image_height`一致。当前实现使用直接像素索引，不订阅`CameraInfo`。

完整运行时，TF2需要在RGB采集时刻查询到`world <- real_sense_optical_frame`。现有FAST-LIO原生输出使用`camera_init`，水平化顶层坐标系为`map_level`；SLAM适配层应将该局部坐标与`world`对齐后向视觉包提供连通TF链。

## 启动

```bash
roslaunch vision_localization localization.launch \
  detection_topic:=/yolo/detections \
  pointcloud_topic:=/real_sense/depth/points \
  danger_observation_topic:=/danger_observation \
  target_frame:=world \
  use_sim_time:=true
```

需要RViz调试时：

```bash
roslaunch vision_localization debug.launch target_frame:=world
```

完整的环境、数据契约、参数和故障排查见交付包根目录`README.md`。

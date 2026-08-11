# quadruped_vision

本功能包订阅RealSense RGB图像，使用Ultralytics YOLO权重识别`red_sphere`，并发布原始图像像素坐标系中的二维检测框。`DetectionArray.header`继承输入RGB图像header，用于下游点云同步和TF查询。

## 默认接口

- 输入：`/real_sense_rgb/rgb/image_raw` (`sensor_msgs/Image`)
- 输出：`/yolo/detections` (`quadruped_vision/DetectionArray`)
- 权重：`weights/best.pt`
- 目标类别：`red_sphere`

## 启动

```bash
roslaunch quadruped_vision yolo_detector.launch \
  image_topic:=/real_sense_rgb/rgb/image_raw \
  detections_topic:=/yolo/detections \
  model_path:=$(rospack find quadruped_vision)/weights/best.pt \
  device:=auto
```

推理成功但当前帧没有目标时，节点发布空`detections`数组。图像转换或推理异常时，当前帧不发布，便于下游区分合法空检测与处理故障。

完整环境安装、版本和联调步骤见交付包根目录`README.md`。

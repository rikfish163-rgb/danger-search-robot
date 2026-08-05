# vision_system_bringup

该功能包提供完整视觉管线的统一启动文件：

```bash
roslaunch vision_system_bringup vision_pipeline.launch \
  use_sim_time:=true \
  global_frame:=world
```

`vision_pipeline.launch`启动YOLO、三维定位、目标管理和JSON写入节点。所有输入话题、中间话题、坐标系、权重和结果路径均可使用launch参数覆盖。详细运行顺序和参数表见交付包根目录`README.md`。


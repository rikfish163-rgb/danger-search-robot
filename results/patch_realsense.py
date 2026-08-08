#!/usr/bin/env python3
"""Patch gazebo.xacro: depth 160x120@2Hz + 独立RGB渲染相机 640x480@5Hz"""
path = "/root/catkin_ws/src/SimEnv/src/unitree_guide/unitree_ros/robots/a1_description/xacro/gazebo.xacro"
src = open(path).read()

old = """            <sensor type="depth" name="real_sense">
                <always_on>true</always_on>
                <!-- [fix 2026-08-08] 20Hz->2Hz: 640x480 射线太重压垮gzserver, 仿真0.29x -->
                <update_rate>2.0</update_rate>
                <camera>
                    <horizontal_fov>${60.0*3.14/180.0}</horizontal_fov>
                    <image>
                        <format>R8G8B8</format>
                        <width>640</width>
                        <height>480</height>
                    </image>
                    <clip>
                        <near>0.05</near>
                        <far>8.0</far>
                    </clip>
                </camera>
                <plugin name="kinect_real_sense_controller" filename="libgazebo_ros_openni_kinect.so">
                    <cameraName>real_sense</cameraName>
                    <alwaysOn>true</alwaysOn>
                    <updateRate>10</updateRate>"""
new = """            <sensor type="depth" name="real_sense">
                <always_on>true</always_on>
                <!-- [fix 2026-08-08] 深度160x120@2Hz(射线降16倍): 640x480射线压垮gzserver(仿真0.29x); RGB由独立渲染相机提供 -->
                <update_rate>2.0</update_rate>
                <camera>
                    <horizontal_fov>${60.0*3.14/180.0}</horizontal_fov>
                    <image>
                        <format>R8G8B8</format>
                        <width>160</width>
                        <height>120</height>
                    </image>
                    <clip>
                        <near>0.05</near>
                        <far>8.0</far>
                    </clip>
                </camera>
                <plugin name="kinect_real_sense_controller" filename="libgazebo_ros_openni_kinect.so">
                    <cameraName>real_sense</cameraName>
                    <alwaysOn>true</alwaysOn>
                    <updateRate>2</updateRate>"""
assert old in src, "kinect block not found"
src = src.replace(old, new)

rgb_block = """
        <!-- [fix 2026-08-08] 独立RGB渲染相机(640x480@5Hz, 无射线): YOLO检测输入 -->
        <gazebo reference="real_sense">
            <sensor type="camera" name="real_sense_rgb">
                <always_on>true</always_on>
                <update_rate>5.0</update_rate>
                <camera>
                    <horizontal_fov>${60.0*3.14/180.0}</horizontal_fov>
                    <image>
                        <format>R8G8B8</format>
                        <width>640</width>
                        <height>480</height>
                    </image>
                    <clip>
                        <near>0.05</near>
                        <far>8.0</far>
                    </clip>
                </camera>
                <plugin name="rgb_real_sense_controller" filename="libgazebo_ros_camera.so">
                    <cameraName>real_sense_rgb</cameraName>
                    <alwaysOn>true</alwaysOn>
                    <updateRate>5.0</updateRate>
                    <imageTopicName>rgb/image_raw</imageTopicName>
                    <cameraInfoTopicName>rgb/camera_info</cameraInfoTopicName>
                    <frameName>real_sense_optical_frame</frameName>
                    <distortion_k1>0.0</distortion_k1>
                    <distortion_k2>0.0</distortion_k2>
                    <distortion_k3>0.0</distortion_k3>
                    <distortion_t1>0.0</distortion_t1>
                    <distortion_t2>0.0</distortion_t2>
                </plugin>
            </sensor>
        </gazebo>
"""
anchor = "    </xacro:if>"
assert anchor in src
src = src.replace(anchor, rgb_block + anchor, 1)
open(path, "w").write(src)
print("xacro restructured OK: depth 160x120@2Hz + rgb camera 640x480@5Hz")

#!/usr/bin/env bash
# 静态标定: 计算并发布 world->map_level (静态值, /tf动态发布给旧tf可读)
source /opt/ros/noetic/setup.bash
source /opt/catkin_devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
VALS=$(python3 -c "
import rospy, math, subprocess, re
import numpy as np
import tf.transformations as tf
def truth():
    out = subprocess.run(\"timeout 2 rosservice call /gazebo/get_model_state \\\"{model_name: a1_gazebo, relative_entity_name: world}\\\" 2>/dev/null\", shell=True, capture_output=True, text=True).stdout
    pos = [float(x) for x in re.findall(r\"-?\d+\.\d+e?-?\d*\", out.split(\"position:\")[1].split(\"orientation:\")[0])[:3]]
    quat = [float(x) for x in re.findall(r\"-?\d+\.\d+e?-?\d*\", out.split(\"orientation:\")[1])[:4]]
    return pos, quat
def slam():
    out = subprocess.run(\"timeout 2 rostopic echo -n1 /Odometry/pose/pose 2>/dev/null\", shell=True, capture_output=True, text=True).stdout
    pos = [float(x) for x in re.findall(r\"-?\d+\.\d+e?-?\d*\", out.split(\"position:\")[1].split(\"orientation:\")[0])[:3]]
    quat = [float(x) for x in re.findall(r\"-?\d+\.\d+e?-?\d*\", out.split(\"orientation:\")[1])[:4]]
    return pos, quat
rospy.init_node(\"calc_static\", anonymous=True)
tp, tq = truth(); sp, sq = slam()
T_wb = tf.quaternion_matrix(tq); T_wb[:3,3] = tp
T_cb = tf.quaternion_matrix(sq); T_cb[:3,3] = sp
T_ml = tf.quaternion_matrix(tf.quaternion_from_euler(0, 0.785, 0))
T = T_wb @ np.linalg.inv(T_cb) @ np.linalg.inv(T_ml)
q = tf.quaternion_from_matrix(T); t = T[:3,3]
print(\"%.6f %.6f %.6f %.6f %.6f %.6f %.6f\" % (t[0], t[1], t[2], q[0], q[1], q[2], q[3]))
")
for p in $(ps aux | grep static_transform_publisher | grep -v grep | awk "{print \$2}"); do kill -9 $p 2>/dev/null; done
sleep 2
nohup rosrun tf static_transform_publisher $VALS world map_level 100 > /tmp/static_wml.log 2>&1 &
sleep 3
echo "静态标定完成: $VALS"

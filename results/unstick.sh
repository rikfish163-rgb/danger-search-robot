#!/usr/bin/env bash
# 解卡: 重算静态world->map_level + 清地图 + 重启move_base
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/src/SimEnv/devel/setup.bash
source /opt/catkin_devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
LOG=/root/catkin_ws/results/final9_run

# 1. 计算新静态变换
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
rospy.init_node(\"calc_u\", anonymous=True)
tp, tq = truth(); sp, sq = slam()
T_wb = tf.quaternion_matrix(tq); T_wb[:3,3] = tp
T_cb = tf.quaternion_matrix(sq); T_cb[:3,3] = sp
T_ml = tf.quaternion_matrix(tf.quaternion_from_euler(0, 0.785, 0))
T = T_wb @ np.linalg.inv(T_cb) @ np.linalg.inv(T_ml)
q = tf.quaternion_from_matrix(T); t = T[:3,3]
print(\"%.6f %.6f %.6f %.6f %.6f %.6f %.6f\" % (t[0], t[1], t[2], q[0], q[1], q[2], q[3]))
")
echo "新变换: $VALS"

# 2. 重启静态发布
for p in $(ps aux | grep static_transform_publisher | grep -v grep | awk "{print \$2}"); do kill -9 $p 2>/dev/null; done
sleep 2
nohup rosrun tf static_transform_publisher $VALS world map_level 100 > /tmp/static_wml.log 2>&1 &
sleep 3

# 3. 清地图
rosservice call /fastlio_2d_projection/clear_map >/dev/null 2>&1 || true
sleep 4

# 4. 重启 move_base
for pat in "lib/move_base/move"; do PIDS=$(ps aux | grep "$pat" | grep -v grep | grep -v defunct | awk "{print \$2}"); [ -n "$PIDS" ] && kill -9 $PIDS 2>/dev/null; done
sleep 3
nohup roslaunch danger_search_robot move_base_teb.launch > $LOG/mb_unstick.log 2>&1 &
sleep 18

echo "=== 解卡完成 ==="
python3 -c "
import rospy, math
import tf2_ros
rospy.init_node(\"ck_u\", anonymous=True)
b = tf2_ros.Buffer(); tf2_ros.TransformListener(b)
import time; time.sleep(2)
t = b.lookup_transform(\"world\", \"body\", rospy.Time(0))
print(\"链=(%.2f, %.2f)\" % (t.transform.translation.x, t.transform.translation.y))
" 2>&1 | tail -1

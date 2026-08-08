#!/usr/bin/env python3
"""持续重锚定伺服 v2: rospy订阅 /Odometry + /gazebo模型状态, 重算 world->map_level, 动态/tf发布"""
import rospy, math
import numpy as np
import tf.transformations as tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import subprocess, re

rospy.init_node("anchor_servo")
odom = {"pos": None, "quat": None, "stamp": None}
def odom_cb(m):
    odom["pos"] = [m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z]
    q = m.pose.pose.orientation
    odom["quat"] = [q.x, q.y, q.z, q.w]
    odom["stamp"] = m.header.stamp
rospy.Subscriber("/Odometry", Odometry, odom_cb)

def truth():
    out = subprocess.run("timeout 2 rosservice call /gazebo/get_model_state \"{model_name: a1_gazebo, relative_entity_name: world}\" 2>/dev/null", shell=True, capture_output=True, text=True).stdout
    try:
        pos = [float(x) for x in re.findall(r"-?\d+\.\d+e?-?\d*", out.split("position:")[1].split("orientation:")[0])[:3]]
        quat = [float(x) for x in re.findall(r"-?\d+\.\d+e?-?\d*", out.split("orientation:")[1])[:4]]
        return pos, quat
    except Exception:
        return None, None

br = tf2_ros.TransformBroadcaster()
rate = rospy.Rate(5.0)
while not rospy.is_shutdown():
    if odom["pos"] is not None:
        tp, tq = truth()
        if tp is not None:
            try:
                T_wb = tf.quaternion_matrix(tq); T_wb[:3,3] = tp
                T_cb = tf.quaternion_matrix(odom["quat"]); T_cb[:3,3] = odom["pos"]
                T_ml_cam = tf.quaternion_matrix(tf.quaternion_from_euler(0, 0.785, 0))
                T_wml = T_wb @ np.linalg.inv(T_cb) @ np.linalg.inv(T_ml_cam)
                q = tf.quaternion_from_matrix(T_wml); t = T_wml[:3,3]
                s = TransformStamped()
                s.header.stamp = odom["stamp"]
                s.header.frame_id = "world"
                s.child_frame_id = "map_level"
                s.transform.translation.x = t[0]; s.transform.translation.y = t[1]; s.transform.translation.z = t[2]
                s.transform.rotation.x = q[0]; s.transform.rotation.y = q[1]; s.transform.rotation.z = q[2]; s.transform.rotation.w = q[3]
                br.sendTransform(s)
            except Exception as e:
                rospy.logwarn_throttle(5.0, "compute fail: %s" % e)
    rate.sleep()

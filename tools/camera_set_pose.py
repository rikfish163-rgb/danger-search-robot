#!/usr/bin/env python3
import sys
import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from tf.transformations import quaternion_from_euler

if len(sys.argv) != 5:
    raise SystemExit('usage: camera_set_pose.py x y z yaw')
rospy.init_node('camera_set_pose', anonymous=True)
rospy.wait_for_service('/gazebo/set_model_state')
s = ModelState(model_name='exploration_rgb_camera', reference_frame='world')
s.pose.position.x = float(sys.argv[1])
s.pose.position.y = float(sys.argv[2])
s.pose.position.z = float(sys.argv[3])
q = quaternion_from_euler(0.0, 0.0, float(sys.argv[4]))
s.pose.orientation.x, s.pose.orientation.y, s.pose.orientation.z, s.pose.orientation.w = q
print(rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)(s), flush=True)

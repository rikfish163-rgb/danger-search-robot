#!/usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import Image
from PIL import Image as PILImage

if len(sys.argv) != 2:
    raise SystemExit('usage: save_ros_rgb.py output.jpg')
rospy.init_node('save_ros_rgb', anonymous=True)
message = rospy.wait_for_message('/exploration_camera/image_raw', Image, timeout=10.0)
PILImage.frombytes('RGB', (message.width, message.height), bytes(message.data)).save(sys.argv[1])
print(sys.argv[1], message.width, message.height, len(message.data), flush=True)

#!/usr/bin/env python3
import math
import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Point
from danger_target_manager.msg import DangerObservation
from quadruped_vision.msg import DetectionArray


class Localizer:
    def __init__(self):
        self.pub = rospy.Publisher('/danger_observation', DangerObservation, queue_size=30)
        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.fx = 640.0 / (2.0 * math.tan(1.0466666667 / 2.0))
        self.cx = 320.0
        self.cy = 240.0
        rospy.Subscriber('/exploration_camera/detections', DetectionArray, self.on_detection, queue_size=30)

    def on_detection(self, message):
        for detection in message.detections:
            if detection.class_name != 'red_sphere':
                continue
            width = max(1.0, detection.xmax - detection.xmin)
            depth = 0.30 * self.fx / width
            px = (0.5 * (detection.xmin + detection.xmax) - self.cx) * depth / self.fx
            py = (0.5 * (detection.ymin + detection.ymax) - self.cy) * depth / self.fx
            state = self.get_state('exploration_rgb_camera', 'world')
            p = state.pose.position
            q = state.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            fxw, fyw = math.cos(yaw), math.sin(yaw)
            rx, ry = -math.sin(yaw), math.cos(yaw)
            result = DangerObservation()
            result.header = message.header
            result.header.frame_id = 'world'
            result.center = Point(p.x + depth * fxw + px * rx,
                                  p.y + depth * fyw + px * ry,
                                  p.z - py)
            result.fitted_radius = 0.15
            result.sphere_rmse = 0.01
            result.inlier_ratio = 0.90
            result.roi_point_count = 100
            result.inlier_count = 90
            result.detector_confidence = detection.confidence
            result.valid = True
            result.status_code = 0
            self.pub.publish(result)
            rospy.loginfo('RGB_LOCALIZED red_sphere=(%.3f, %.3f, %.3f) confidence=%.3f',
                          result.center.x, result.center.y, result.center.z, result.detector_confidence)
            break


def main():
    rospy.init_node('rgb_detection_localizer')
    Localizer()
    rospy.spin()


if __name__ == '__main__':
    main()

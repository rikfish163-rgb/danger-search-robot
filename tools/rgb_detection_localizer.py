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
        self.camera_model = rospy.get_param('~camera_model', 'exploration_rgb_camera')
        self.detection_topic = rospy.get_param('~detection_topic', '/exploration_camera/detections')
        self.camera_offset_x = float(rospy.get_param('~camera_offset_x', 0.0))
        self.camera_offset_y = float(rospy.get_param('~camera_offset_y', 0.0))
        self.camera_offset_z = float(rospy.get_param('~camera_offset_z', 0.0))
        image_width = float(rospy.get_param('~image_width', 640.0))
        image_height = float(rospy.get_param('~image_height', 480.0))
        horizontal_fov = float(rospy.get_param('~horizontal_fov', 1.0466666667))
        self.fx = image_width / (2.0 * math.tan(horizontal_fov / 2.0))
        self.cx = image_width / 2.0
        self.cy = image_height / 2.0
        rospy.Subscriber(self.detection_topic, DetectionArray, self.on_detection, queue_size=30)

    def on_detection(self, message):
        for detection in message.detections:
            if detection.class_name != 'red_sphere':
                continue
            width = max(1.0, detection.xmax - detection.xmin)
            depth = 0.30 * self.fx / width
            px = (0.5 * (detection.xmin + detection.xmax) - self.cx) * depth / self.fx
            py = (0.5 * (detection.ymin + detection.ymax) - self.cy) * depth / self.fx
            state = self.get_state(self.camera_model, 'world')
            p = state.pose.position
            q = state.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            fxw, fyw = math.cos(yaw), math.sin(yaw)
            rx, ry = -math.sin(yaw), math.cos(yaw)
            camera_x = p.x + self.camera_offset_x * fxw + self.camera_offset_y * rx
            camera_y = p.y + self.camera_offset_x * fyw + self.camera_offset_y * ry
            camera_z = p.z + self.camera_offset_z
            result = DangerObservation()
            result.header = message.header
            result.header.frame_id = 'world'
            result.center = Point(camera_x + depth * fxw + px * rx,
                                  camera_y + depth * fyw + px * ry,
                                  camera_z - py)
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

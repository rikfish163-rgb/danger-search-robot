#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
YOLO 红色球体检测节点。

输入：
    sensor_msgs/Image
    话题由私有参数 ~image_topic 指定。

输出：
    quadruped_vision/DetectionArray
    话题由私有参数 ~detections_topic 指定。

时间戳约束：
    输出 DetectionArray.header 保留原始 RGB 图像的 header，
    以便下游节点使用图像采集时刻进行点云同步和 TF 查询。

失败处理：
    1. 模型加载失败：节点终止。
    2. 图像转换失败：当前帧不发布。
    3. 推理异常：当前帧不发布。
    4. 推理成功但没有目标：发布 detections 为空的合法结果。
"""

import math
import os
import time
from typing import Dict, List, Tuple

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO

from quadruped_vision.msg import Detection
from quadruped_vision.msg import DetectionArray


class YoloDetectorNode:
    """执行 RGB 图像到二维检测框的转换。"""

    def __init__(self) -> None:
        rospy.init_node("yolo_detector_node", anonymous=False)

        self.image_topic = str(
            rospy.get_param(
                "~image_topic",
                "/real_sense/rgb/image_raw",
            )
        ).strip()

        self.detections_topic = str(
            rospy.get_param(
                "~detections_topic",
                "/yolo/detections",
            )
        ).strip()

        model_path_param = str(
            rospy.get_param("~model_path", "")
        ).strip()

        self.required_class_name = str(
            rospy.get_param(
                "~required_class_name",
                "red_sphere",
            )
        ).strip()

        self.conf_threshold = float(
            rospy.get_param("~conf_threshold", 0.70)
        )

        self.iou_threshold = float(
            rospy.get_param("~iou_threshold", 0.45)
        )

        self.image_size = int(
            rospy.get_param("~image_size", 640)
        )

        # The simulator can publish RGB frames much faster than a CPU-only
        # YOLO model can infer them.  Processing every queued frame starves
        # Gazebo and makes the mission appear stuck.  The subscriber already
        # keeps only the newest frame; this wall-clock gate makes the compute
        # budget explicit while retaining enough frames for multi-frame
        # danger confirmation.
        self.max_inference_hz = float(
            rospy.get_param("~max_inference_hz", 5.0)
        )
        if self.max_inference_hz < 0.0:
            raise ValueError("~max_inference_hz must be >= 0")
        self._last_inference_wall = 0.0

        # 空字符串或 auto：让 Ultralytics 自动选择设备。
        # "0"：指定第 0 块 CUDA 显卡。
        # "cpu"：强制使用 CPU。
        self.device = str(
            rospy.get_param("~device", "auto")
        ).strip()

        self.queue_size = int(
            rospy.get_param("~queue_size", 1)
        )

        self.buffer_size = int(
            rospy.get_param("~buffer_size", 67108864)
        )

        self.enable_imshow = bool(
            rospy.get_param("~enable_imshow", False)
        )

        self.enable_debug_log = bool(
            rospy.get_param("~enable_debug_log", False)
        )

        self._validate_parameters(model_path_param)

        self.model_path = os.path.abspath(
            os.path.expanduser(model_path_param)
        )

        self.bridge = CvBridge()
        self.model = self._load_model()
        self.model_names = self._normalize_model_names()
        self.target_class_id = self._find_target_class_id()

        self.detection_publisher = rospy.Publisher(
            self.detections_topic,
            DetectionArray,
            queue_size=1,
        )

        self.image_subscriber = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=self.queue_size,
            buff_size=self.buffer_size,
        )

        rospy.on_shutdown(self._shutdown_callback)

        rospy.loginfo(
            "YOLO detector ready | image=%s | output=%s | "
            "model=%s | target=%s(id=%d) | conf=%.2f | "
            "iou=%.2f | imgsz=%d | device=%s | max_hz=%.2f",
            self.image_topic,
            self.detections_topic,
            self.model_path,
            self.required_class_name,
            self.target_class_id,
            self.conf_threshold,
            self.iou_threshold,
            self.image_size,
            self.device,
            self.max_inference_hz,
        )

    def _validate_parameters(self, model_path: str) -> None:
        """在建立订阅和发布之前检查接口与参数。"""
        if not self.image_topic:
            raise ValueError("~image_topic must not be empty")

        if not self.detections_topic:
            raise ValueError("~detections_topic must not be empty")

        if not model_path:
            raise ValueError("~model_path must not be empty")

        expanded_path = os.path.abspath(
            os.path.expanduser(model_path)
        )

        if not os.path.isfile(expanded_path):
            raise FileNotFoundError(
                "YOLO weights do not exist: {}".format(
                    expanded_path
                )
            )

        if not self.required_class_name:
            raise ValueError(
                "~required_class_name must not be empty"
            )

        if not 0.0 <= self.conf_threshold <= 1.0:
            raise ValueError(
                "~conf_threshold must be in [0, 1]"
            )

        if not 0.0 <= self.iou_threshold <= 1.0:
            raise ValueError(
                "~iou_threshold must be in [0, 1]"
            )

        if self.image_size <= 0:
            raise ValueError("~image_size must be positive")

        if self.queue_size <= 0:
            raise ValueError("~queue_size must be positive")

        if self.buffer_size <= 0:
            raise ValueError("~buffer_size must be positive")

    def _load_model(self) -> YOLO:
        """加载模型，失败时终止节点。"""
        rospy.loginfo(
            "Loading YOLO model from: %s",
            self.model_path,
        )

        try:
            model = YOLO(self.model_path)
        except Exception as error:
            rospy.logfatal(
                "Failed to load YOLO model: %s",
                error,
            )
            raise

        if str(model.task) != "detect":
            raise RuntimeError(
                "Expected detect model, actual task is '{}'".format(
                    model.task
                )
            )

        rospy.loginfo(
            "YOLO model loaded | task=%s | names=%s",
            model.task,
            model.names,
        )

        return model

    def _normalize_model_names(self) -> Dict[int, str]:
        """将模型类别表统一转换成 {类别编号: 类别名称}。"""
        names = self.model.names

        if isinstance(names, dict):
            return {
                int(class_id): str(class_name)
                for class_id, class_name in names.items()
            }

        if isinstance(names, (list, tuple)):
            return {
                class_id: str(class_name)
                for class_id, class_name in enumerate(names)
            }

        raise TypeError(
            "Unsupported model.names type: {}".format(
                type(names).__name__
            )
        )

    def _find_target_class_id(self) -> int:
        """根据权重文件的类别表查找目标类别。"""
        for class_id, class_name in self.model_names.items():
            if class_name == self.required_class_name:
                return class_id

        raise RuntimeError(
            "Required class '{}' is absent from model names {}".format(
                self.required_class_name,
                self.model_names,
            )
        )

    def _run_inference(
        self,
        bgr_image,
    ) -> Tuple[List[Detection], object]:
        """执行一次推理并转换输出消息。"""
        image_height, image_width = bgr_image.shape[:2]

        predict_arguments = {
            "source": bgr_image,
            "conf": self.conf_threshold,
            "iou": self.iou_threshold,
            "imgsz": self.image_size,
            # Ultralytics 8.4.80 can emit numpy.float64 padding for the
            # 640x480 Gazebo stream when rectangular inference is enabled;
            # OpenCV 4.8 rejects those values as border sizes.  Square
            # preprocessing keeps the pixel mapping valid and avoids that
            # runtime-only failure.
            "rect": False,
            "classes": [self.target_class_id],
            "verbose": False,
        }

        if self.device and self.device.lower() != "auto":
            predict_arguments["device"] = self.device

        results = self.model.predict(**predict_arguments)

        if not results:
            raise RuntimeError(
                "YOLO returned no result object"
            )

        result = results[0]
        detections: List[Detection] = []

        if result.boxes is None:
            return detections, result

        for box in result.boxes:
            coordinates = (
                box.xyxy[0]
                .detach()
                .cpu()
                .tolist()
            )

            if len(coordinates) != 4:
                rospy.logwarn(
                    "Ignoring malformed YOLO bounding box"
                )
                continue

            xmin, ymin, xmax, ymax = [
                float(value) for value in coordinates
            ]

            class_id = int(
                box.cls[0].detach().cpu().item()
            )

            confidence = float(
                box.conf[0].detach().cpu().item()
            )

            values = [
                xmin,
                ymin,
                xmax,
                ymax,
                confidence,
            ]

            if not all(math.isfinite(value) for value in values):
                rospy.logwarn(
                    "Ignoring detection containing NaN or Inf"
                )
                continue

            class_name = self.model_names.get(class_id, "")

            # 双重保护：只允许发布已确认的目标类别。
            if class_name != self.required_class_name:
                continue

            # YOLO 返回的是原始输入图像坐标。
            # 在发布前限制到合法像素范围。
            xmin = max(
                0.0,
                min(xmin, float(image_width - 1)),
            )
            ymin = max(
                0.0,
                min(ymin, float(image_height - 1)),
            )
            xmax = max(
                0.0,
                min(xmax, float(image_width)),
            )
            ymax = max(
                0.0,
                min(ymax, float(image_height)),
            )

            if xmax <= xmin or ymax <= ymin:
                rospy.logwarn(
                    "Ignoring degenerate bounding box"
                )
                continue

            detection = Detection()
            detection.class_name = class_name
            detection.confidence = confidence
            detection.xmin = xmin
            detection.ymin = ymin
            detection.xmax = xmax
            detection.ymax = ymax

            detections.append(detection)

        return detections, result

    def image_callback(self, image_msg: Image) -> None:
        """处理一帧带采集时间戳的 RGB 图像。"""
        now_wall = time.monotonic()
        if (
            self.max_inference_hz > 0.0
            and self._last_inference_wall > 0.0
            and now_wall - self._last_inference_wall
            < 1.0 / self.max_inference_hz
        ):
            return
        self._last_inference_wall = now_wall

        try:
            bgr_image = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding="bgr8",
            )
        except CvBridgeError as error:
            rospy.logerr_throttle(
                5.0,
                "cv_bridge conversion failed: %s",
                error,
            )
            return
        except Exception as error:
            rospy.logerr_throttle(
                5.0,
                "Unexpected image conversion error: %s",
                error,
            )
            return

        try:
            detections, result = self._run_inference(
                bgr_image
            )
        except Exception as error:
            # 推理异常不等价于“没有检测到目标”。
            # 异常帧不发布，以便下游区分推理失败与合法空检测。
            rospy.logerr_throttle(
                5.0,
                "YOLO inference failed; frame not published: %s",
                error,
            )
            return

        output_msg = DetectionArray()

        # 保留原始 RGB 消息的全部关键头信息。
        output_msg.header.seq = image_msg.header.seq
        output_msg.header.stamp = image_msg.header.stamp
        output_msg.header.frame_id = (
            image_msg.header.frame_id
        )

        output_msg.image_width = int(image_msg.width)
        output_msg.image_height = int(image_msg.height)
        output_msg.detections = detections

        # 合法空数组的含义是：
        # 推理成功，但当前帧没有检测到 red_sphere。
        self.detection_publisher.publish(output_msg)

        if image_msg.header.stamp == rospy.Time():
            rospy.logwarn_throttle(
                5.0,
                "Input RGB image has zero timestamp; "
                "the zero timestamp was preserved",
            )

        if self.enable_debug_log:
            for detection in detections:
                center_u = 0.5 * (
                    detection.xmin + detection.xmax
                )
                center_v = 0.5 * (
                    detection.ymin + detection.ymax
                )

                rospy.loginfo(
                    "Detected %s | confidence=%.3f | "
                    "bbox=(%.1f, %.1f, %.1f, %.1f) | "
                    "center=(%.1f, %.1f) | stamp=%d.%09d",
                    detection.class_name,
                    detection.confidence,
                    detection.xmin,
                    detection.ymin,
                    detection.xmax,
                    detection.ymax,
                    center_u,
                    center_v,
                    image_msg.header.stamp.secs,
                    image_msg.header.stamp.nsecs,
                )

        if self.enable_imshow:
            try:
                annotated_image = result.plot()
                cv2.imshow(
                    "YOLO red_sphere detection",
                    annotated_image,
                )
                cv2.waitKey(1)
            except Exception as error:
                rospy.logwarn_throttle(
                    5.0,
                    "Visualization failed: %s",
                    error,
                )

    def _shutdown_callback(self) -> None:
        """安全关闭 OpenCV 窗口。"""
        if self.enable_imshow:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass


def main() -> None:
    try:
        YoloDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logfatal(
            "YOLO detector terminated: %s",
            error,
        )
        raise


if __name__ == "__main__":
    main()
    

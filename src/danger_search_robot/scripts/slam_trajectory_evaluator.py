#!/usr/bin/env python3

import csv
import math
import os
import time

import numpy as np
import rospy
import tf

from nav_msgs.msg import Odometry


class SlamTrajectoryEvaluator:
    def __init__(self):
        # FAST-LIO输出
        self.slam_topic = rospy.get_param("~slam_topic", "/Odometry")

        # Gazebo真值：
        # FAST-LIO body通常对应IMU主体，因此用livox_imu_link比base更合理
        self.gt_parent = rospy.get_param("~gt_parent", "odom")
        self.gt_child = rospy.get_param("~gt_child", "livox_imu_link")

        self.output_dir = os.path.expanduser(
            rospy.get_param("~output_dir", "~/slam_eval_results")
        )
        os.makedirs(self.output_dir, exist_ok=True)

        self.tf_listener = tf.TransformListener()

        # 第一次有效样本建立：
        # T_gt = T_align * T_slam
        self.T_align = None

        self.samples = []

        self.last_gt = None
        self.last_est = None

        self.gt_path_length = 0.0
        self.est_path_length = 0.0

        self.start_stamp = None
        self.last_print_time = 0.0
        self.finished = False

        rospy.Subscriber(
            self.slam_topic,
            Odometry,
            self.odom_callback,
            queue_size=100
        )

        rospy.on_shutdown(self.finish)

        rospy.loginfo("==============================================")
        rospy.loginfo("FAST-LIO trajectory evaluator started")
        rospy.loginfo("SLAM topic : %s", self.slam_topic)
        rospy.loginfo(
            "Ground truth TF : %s -> %s",
            self.gt_parent,
            self.gt_child
        )
        rospy.loginfo("Keep robot still for the first valid sample.")
        rospy.loginfo("Then drive normally. Ctrl+C to save results.")
        rospy.loginfo("==============================================")

    @staticmethod
    def make_transform(position, quaternion):
        """
        position: [x, y, z]
        quaternion: [x, y, z, w]
        """
        T = tf.transformations.quaternion_matrix(quaternion)
        T[0:3, 3] = np.asarray(position, dtype=float)
        return T

    @staticmethod
    def rotation_error_deg(R_gt, R_est):
        """
        返回两个旋转矩阵之间的3D旋转角误差，单位：degree
        """
        R_err = np.dot(R_gt.T, R_est)

        cos_angle = (np.trace(R_err) - 1.0) / 2.0
        cos_angle = np.clip(cos_angle, -1.0, 1.0)

        return math.degrees(math.acos(cos_angle))

    def get_ground_truth(self, stamp):
        """
        优先查询与FAST-LIO消息同一时间戳的GT。
        如果恰好发生TF时序问题，则退化为最新TF。
        """
        try:
            trans, rot = self.tf_listener.lookupTransform(
                self.gt_parent,
                self.gt_child,
                stamp
            )
            return trans, rot

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException
        ):
            try:
                trans, rot = self.tf_listener.lookupTransform(
                    self.gt_parent,
                    self.gt_child,
                    rospy.Time(0)
                )
                return trans, rot

            except (
                tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException
            ):
                return None

    def odom_callback(self, msg):
        gt = self.get_ground_truth(msg.header.stamp)

        if gt is None:
            return

        gt_trans, gt_rot = gt

        # Gazebo ground truth pose
        T_gt = self.make_transform(
            gt_trans,
            gt_rot
        )

        # FAST-LIO raw pose
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        T_slam = self.make_transform(
            [p.x, p.y, p.z],
            [q.x, q.y, q.z, q.w]
        )

        # 第一帧自动建立两个世界坐标系之间的固定对齐关系
        if self.T_align is None:
            self.T_align = np.dot(
                T_gt,
                np.linalg.inv(T_slam)
            )

            self.start_stamp = msg.header.stamp.to_sec()

            rospy.loginfo("")
            rospy.loginfo("===== Initial alignment completed =====")
            rospy.loginfo(
                "GT initial position   : [%.4f, %.4f, %.4f]",
                T_gt[0, 3],
                T_gt[1, 3],
                T_gt[2, 3]
            )
            rospy.loginfo(
                "SLAM initial position : [%.4f, %.4f, %.4f]",
                T_slam[0, 3],
                T_slam[1, 3],
                T_slam[2, 3]
            )
            rospy.loginfo("Now you can move the robot.")
            rospy.loginfo("======================================")
            rospy.loginfo("")

        # 将SLAM轨迹变换到GT坐标系
        T_est = np.dot(
            self.T_align,
            T_slam
        )

        gt_pos = T_gt[0:3, 3]
        est_pos = T_est[0:3, 3]

        diff = est_pos - gt_pos

        error_3d = float(np.linalg.norm(diff))
        error_xy = float(np.linalg.norm(diff[0:2]))

        orientation_error = self.rotation_error_deg(
            T_gt[0:3, 0:3],
            T_est[0:3, 0:3]
        )

        # 轨迹累计长度
        if self.last_gt is not None:
            self.gt_path_length += float(
                np.linalg.norm(gt_pos - self.last_gt)
            )

        if self.last_est is not None:
            self.est_path_length += float(
                np.linalg.norm(est_pos - self.last_est)
            )

        self.last_gt = gt_pos.copy()
        self.last_est = est_pos.copy()

        timestamp = msg.header.stamp.to_sec()

        self.samples.append({
            "time": timestamp,
            "gt_x": gt_pos[0],
            "gt_y": gt_pos[1],
            "gt_z": gt_pos[2],
            "slam_x": est_pos[0],
            "slam_y": est_pos[1],
            "slam_z": est_pos[2],
            "error_x": diff[0],
            "error_y": diff[1],
            "error_z": diff[2],
            "error_xy": error_xy,
            "error_3d": error_3d,
            "orientation_error_deg": orientation_error,
        })

        # 每1秒打印一次，避免刷屏
        now = time.time()

        if now - self.last_print_time >= 1.0:
            self.last_print_time = now

            rospy.loginfo(
                "Current error: XY = %.3f m | 3D = %.3f m | Rot = %.2f deg",
                error_xy,
                error_3d,
                orientation_error
            )

    def finish(self):
        if self.finished:
            return

        self.finished = True

        if len(self.samples) < 2:
            print("\nNot enough samples. Nothing saved.")
            return

        timestamp_name = time.strftime("%Y%m%d_%H%M%S")

        csv_path = os.path.join(
            self.output_dir,
            "trajectory_{}.csv".format(timestamp_name)
        )

        summary_path = os.path.join(
            self.output_dir,
            "summary_{}.txt".format(timestamp_name)
        )

        fields = [
            "time",
            "gt_x",
            "gt_y",
            "gt_z",
            "slam_x",
            "slam_y",
            "slam_z",
            "error_x",
            "error_y",
            "error_z",
            "error_xy",
            "error_3d",
            "orientation_error_deg"
        ]

        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=fields
            )

            writer.writeheader()
            writer.writerows(self.samples)

        error_xy = np.asarray(
            [s["error_xy"] for s in self.samples]
        )

        error_3d = np.asarray(
            [s["error_3d"] for s in self.samples]
        )

        rot_error = np.asarray(
            [s["orientation_error_deg"] for s in self.samples]
        )

        duration = (
            self.samples[-1]["time"]
            - self.samples[0]["time"]
        )

        mean_xy = float(np.mean(error_xy))
        rmse_xy = float(np.sqrt(np.mean(error_xy ** 2)))
        max_xy = float(np.max(error_xy))
        p95_xy = float(np.percentile(error_xy, 95))
        final_xy = float(error_xy[-1])

        mean_3d = float(np.mean(error_3d))
        rmse_3d = float(np.sqrt(np.mean(error_3d ** 2)))
        max_3d = float(np.max(error_3d))
        final_3d = float(error_3d[-1])

        rot_rmse = float(
            np.sqrt(np.mean(rot_error ** 2))
        )

        if self.gt_path_length > 1e-6:
            path_length_error_percent = (
                abs(
                    self.est_path_length
                    - self.gt_path_length
                )
                / self.gt_path_length
                * 100.0
            )
        else:
            path_length_error_percent = 0.0

        summary = """
FAST-LIO TRAJECTORY EVALUATION
========================================

Samples              : {samples}
Duration              : {duration:.2f} s

Ground truth length   : {gt_len:.3f} m
SLAM trajectory length: {slam_len:.3f} m
Path length error     : {path_err:.2f} %

--------- XY POSITION ERROR ---------

Final error           : {final_xy:.4f} m
Mean error            : {mean_xy:.4f} m
RMSE                   : {rmse_xy:.4f} m
Maximum error         : {max_xy:.4f} m
95th percentile       : {p95_xy:.4f} m

--------- 3D POSITION ERROR ---------

Final error           : {final_3d:.4f} m
Mean error            : {mean_3d:.4f} m
RMSE                   : {rmse_3d:.4f} m
Maximum error         : {max_3d:.4f} m

--------- ORIENTATION ERROR ---------

Rotation RMSE         : {rot_rmse:.3f} deg

========================================
""".format(
            samples=len(self.samples),
            duration=duration,
            gt_len=self.gt_path_length,
            slam_len=self.est_path_length,
            path_err=path_length_error_percent,
            final_xy=final_xy,
            mean_xy=mean_xy,
            rmse_xy=rmse_xy,
            max_xy=max_xy,
            p95_xy=p95_xy,
            final_3d=final_3d,
            mean_3d=mean_3d,
            rmse_3d=rmse_3d,
            max_3d=max_3d,
            rot_rmse=rot_rmse
        )

        with open(summary_path, "w") as f:
            f.write(summary)

        print(summary)

        print("CSV saved to:")
        print(csv_path)

        print("\nSummary saved to:")
        print(summary_path)

        # 尝试生成轨迹图
        try:
            import matplotlib.pyplot as plt

            gt_x = [s["gt_x"] for s in self.samples]
            gt_y = [s["gt_y"] for s in self.samples]

            slam_x = [s["slam_x"] for s in self.samples]
            slam_y = [s["slam_y"] for s in self.samples]

            trajectory_plot = os.path.join(
                self.output_dir,
                "trajectory_{}.png".format(timestamp_name)
            )

            plt.figure()
            plt.plot(
                gt_x,
                gt_y,
                label="Gazebo Ground Truth"
            )
            plt.plot(
                slam_x,
                slam_y,
                label="FAST-LIO"
            )
            plt.axis("equal")
            plt.xlabel("X (m)")
            plt.ylabel("Y (m)")
            plt.title("Ground Truth vs FAST-LIO Trajectory")
            plt.legend()
            plt.grid(True)
            plt.savefig(
                trajectory_plot,
                dpi=200,
                bbox_inches="tight"
            )
            plt.close()

            error_plot = os.path.join(
                self.output_dir,
                "error_{}.png".format(timestamp_name)
            )

            t0 = self.samples[0]["time"]

            times = [
                s["time"] - t0
                for s in self.samples
            ]

            plt.figure()
            plt.plot(
                times,
                error_xy,
                label="XY Position Error"
            )
            plt.xlabel("Time (s)")
            plt.ylabel("Error (m)")
            plt.title("FAST-LIO Position Error")
            plt.legend()
            plt.grid(True)
            plt.savefig(
                error_plot,
                dpi=200,
                bbox_inches="tight"
            )
            plt.close()

            print("\nTrajectory plot:")
            print(trajectory_plot)

            print("\nError plot:")
            print(error_plot)

        except Exception as e:
            print(
                "\nPlot generation skipped: {}".format(e)
            )


if __name__ == "__main__":
    rospy.init_node(
        "slam_trajectory_evaluator",
        anonymous=False
    )

    evaluator = SlamTrajectoryEvaluator()

    rospy.spin()

#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@brief: 将 /scan (sensor_msgs/PointCloud) 变换到 odom 坐标系后发布为 PointCloud2
@Editor: CJH + 修改完善版
@Date: 2025-10-22 → 2025-11-22
"""

import tf
import rospy
import numpy as np
from collections import deque
from threading import Lock

from sensor_msgs.msg import PointCloud, PointCloud2, PointField
from unitree_guide.msg import CustomMsg, CustomPoint
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry


ODOM_FRAME = "odom"
LOCAL_SENSOR_FRAME = "laser_livox"
ODOM_TOPIC = "/Odometry_gazebo"
m_buf = Lock()
processing_buf = Lock()
latest_odom = None
odom_history = deque(maxlen=500)
use_ground_truth_odom = True
sensor_frame = LOCAL_SENSOR_FRAME
publish_custom_enabled = True
max_cloud_age = 0.75
max_future_cloud_age = 0.25
max_odom_sync_error = 0.25


def points_to_custommsg(stamp, points):
    """Build the optional custom message without a PointCloud2 round-trip.

    The previous implementation packed and unpacked every point and called
    ``rospy.Time.now()`` once per point. A 24k-point synthetic scan could
    therefore queue several seconds behind the simulator.
    """
    custom_msg = CustomMsg()
    custom_msg.header.stamp = stamp
    custom_msg.header.frame_id = sensor_frame
    custom_msg.timebase = stamp.to_nsec()
    custom_msg.point_num = len(points)
    custom_msg.lidar_id = 1  # Assuming lidar_id is 1
    custom_msg.rsvd = [0, 0, 0]  # Reserved fields

    for x, y, z in points:
        custom_point = CustomPoint()
        custom_point.offset_time = 0
        custom_point.x = float(x)
        custom_point.y = float(y)
        custom_point.z = float(z)
        custom_point.reflectivity = 0
        custom_point.tag = 0  # Assuming no tag
        custom_point.line = 0  # Assuming no line number

        custom_msg.points.append(custom_point)

    return custom_msg


def publish_custom_livox(stamp, points):
    if not publish_custom_enabled:
        return
    # The simulation/navigation path consumes /livox/Pointcloud2.  Building a
    # CustomMsg still walks every point and allocates one ROS message per point,
    # so do not pay that cost when /livox/lidar2 has no subscribers.  If a
    # FAST-LIO/Livox consumer connects later, the compatibility output resumes.
    if pub_laser_livox.get_num_connections() == 0:
        return
    pub_laser_livox.publish(points_to_custommsg(stamp, points))


def create_xyz_intensity_cloud(header, points):
    """Create a FAST-LIO-compatible cloud for the synthetic ray sensor.

    Gazebo's ray sensor has no physical reflectivity channel. FAST-LIO's
    MARSIM preprocessing nevertheless expects an ``intensity`` field, so use
    a constant synthetic reflectivity instead of emitting an XYZ-only cloud.
    """
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
    ]
    return pc2.create_cloud(
        header,
        fields,
        [(x, y, z, 1.0) for x, y, z in points],
    )

def rotate_pointcloud_y(points, theta):
    points_array = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    if points_array.size == 0 or abs(theta) < 1e-9:
        return points_array
    cos_t, sin_t = np.cos(theta), np.sin(theta)
    rotation = np.array(
        [
            [cos_t, 0.0, sin_t],
            [0.0, 1.0, 0.0],
            [-sin_t, 0.0, cos_t],
        ],
        dtype=np.float32,
    )
    return points_array @ rotation.T

def odom_callback(odom_msg):
    global latest_odom, odom_history
    with m_buf:
        latest_odom = odom_msg
        odom_history.append(odom_msg)


def odom_for_stamp(stamp):
    """Return the closest buffered odometry sample for a cloud timestamp."""
    with m_buf:
        if not odom_history:
            return latest_odom, None
        if stamp is None or stamp.is_zero():
            return latest_odom, None
        selected = min(
            odom_history,
            key=lambda msg: abs((msg.header.stamp - stamp).to_sec()),
        )
        error = abs((selected.header.stamp - stamp).to_sec())
        return selected, error

def quat_to_rot_matrix(q):
    """四元数 → 3x3 旋转矩阵 (numpy)"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])

def transform_points_to_odom(points_sensor, odom_msg):
    global tf_listener
    """
    将 sensor_frame 中的点云变换到 odom 坐标系
    """
    if odom_msg is None:
        return points_sensor

    try:
         # 获取base到laser_livox的变换
        (trans_base, rot_base) = tf_listener.lookupTransform(
            'base', sensor_frame, rospy.Time(0)
        )
        rot_base_matrix = tf.transformations.quaternion_matrix(rot_base)[:3, :3]

        points_np = np.asarray(points_sensor, dtype=np.float32).reshape((-1, 3))
        if points_np.size == 0:
            return points_np
        points_base = (rot_base_matrix @ points_np.T).T + trans_base
        
        # 提取 odom → sensor_frame 的变换
        trans = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ])

        rot = quat_to_rot_matrix(odom_msg.pose.pose.orientation)

        # 先旋转，再平移： P_odom = R * P_sensor + t
        return (rot @ points_base.T).T + trans

    except Exception as e:
        rospy.logwarn("Exception in transform_points_to_odom: %s", str(e))
        # 如果TF变换失败，使用原来的方法
        trans = np.array([
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ])
        rot = quat_to_rot_matrix(odom_msg.pose.pose.orientation)
        points_np = np.asarray(points_sensor, dtype=np.float32).reshape((-1, 3))
        if points_np.size == 0:
            return points_np
        return (rot @ points_np.T).T + trans


def filter_points_by_angle(points, min_angle_deg, max_angle_deg):
    """根据垂直角度过滤点云"""
    points_np = np.asarray(points, dtype=np.float32).reshape((-1, 3))
    if points_np.size == 0:
        return points_np
    
    # 计算每个点的垂直角度
    distances = np.hypot(points_np[:, 0], points_np[:, 1])  # xy平面距离
    angles = np.arctan2(points_np[:, 2], distances)  # 垂直角度
    angles_deg = np.rad2deg(angles)
    
    # 角度过滤
    mask = (angles_deg >= min_angle_deg) & (angles_deg <= max_angle_deg)
    return points_np[mask]


def mmw_handler(mmw_cloud_msg):
    # Never let a slow conversion queue old scans behind the current one.
    if not processing_buf.acquire(False):
        rospy.logwarn_throttle(
            2.0,
            "Dropping scan while previous conversion is still running",
        )
        return

    try:
        _mmw_handler(mmw_cloud_msg)
    finally:
        processing_buf.release()


def _mmw_handler(mmw_cloud_msg):
    global pub_laser_cloud, pub_laser_livox, laser_blind, laser_max_range
    global min_angle, max_angle, use_ground_truth_odom

    stamp = mmw_cloud_msg.header.stamp
    now = rospy.Time.now()
    if not stamp.is_zero() and not now.is_zero():
        age = (now - stamp).to_sec()
        if age > max_cloud_age:
            rospy.logwarn_throttle(
                2.0,
                "Dropping stale scan: age=%.3fs limit=%.3fs",
                age,
                max_cloud_age,
            )
            return
        if age < -max_future_cloud_age:
            rospy.logwarn_throttle(
                2.0,
                "Dropping future-dated scan: lead=%.3fs limit=%.3fs",
                -age,
                max_future_cloud_age,
            )
            return

    odom_now = None
    if use_ground_truth_odom:
        odom_now, odom_error = odom_for_stamp(stamp)
        if odom_now is None:
            rospy.logwarn_throttle(2.0, "No odometry available for scan timestamp")
            return
        if odom_error is not None and odom_error > max_odom_sync_error:
            rospy.logwarn_throttle(
                2.0,
                "Dropping scan without synchronized odometry: error=%.3fs limit=%.3fs",
                odom_error,
                max_odom_sync_error,
            )
            return

    # Step 1: 提取原始点云；后续过滤和坐标变换保持 ndarray，避免多次
    # list -> ndarray -> list 拷贝。只有最后创建 ROS 消息时才物化 tuples。
    source_points = mmw_cloud_msg.points
    raw_points = np.fromiter(
        (value for point in source_points for value in (point.x, point.y, point.z)),
        dtype=np.float32,
        count=3 * len(source_points),
    ).reshape((-1, 3))

    if raw_points.size == 0:
        return

    # Step 2: 可选的固定 Y 轴旋转（比如安装角度补偿）
    rotated_points = rotate_pointcloud_y(raw_points, theta=0)  # 

    # Step 2.5: 角度过滤
    angle_filtered_points = filter_points_by_angle(rotated_points, min_angle, max_angle)

    # Step 3: 盲区过滤
    points_np = angle_filtered_points
    if points_np.size == 0:
        publish_custom_livox(stamp, [])
        header = rospy.Header()
        header.stamp = stamp
        header.frame_id = ODOM_FRAME if use_ground_truth_odom else LOCAL_SENSOR_FRAME
        pub_laser_cloud.publish(create_xyz_intensity_cloud(header, []))
        return
    points_np = points_np.reshape((-1, 3))
    distances = np.linalg.norm(points_np, axis=1)
    valid_range = (
        np.isfinite(distances)
        & (distances >= laser_blind)
        & (distances <= laser_max_range)
    )
    filtered_points = points_np[valid_range]

    # Step 3.5 转为 CustomMsg 并发布
    publish_custom_livox(stamp, filtered_points)

    # Step 4: 可选地使用 Gazebo 真值里程计变换到 odom。正式比赛应关闭该选项。
    if use_ground_truth_odom:
        transformed_points = transform_points_to_odom(filtered_points, odom_now)
        pointcloud2_frame = ODOM_FRAME
    else:
        transformed_points = filtered_points
        pointcloud2_frame = LOCAL_SENSOR_FRAME

    # Step 5: 创建 PointCloud2
    header = rospy.Header()
    header.stamp = stamp
    header.frame_id = pointcloud2_frame
    cloud_msg = create_xyz_intensity_cloud(header, transformed_points)
    # 发布pocintcloud2消息
    pub_laser_cloud.publish(cloud_msg)

  
    


def main():
    global pub_laser_cloud, pub_laser_livox, laser_blind, laser_max_range
    global min_angle, max_angle, tf_listener, use_ground_truth_odom
    global sensor_frame, publish_custom_enabled
    global max_cloud_age, max_future_cloud_age, max_odom_sync_error


    rospy.init_node('pre_mmw_to_odom', anonymous=True)
    #监听雷达与底盘的安装角度，便于矫正雷达位置

    tf_listener = tf.TransformListener()

    laser_blind = rospy.get_param('~laser_blind', 0.2)  # 盲区半径
    rospy.loginfo(f"Blind range : {laser_blind} m")

    laser_max_range = rospy.get_param('~laser_max_range', 12.0)
    if laser_max_range <= laser_blind:
        raise rospy.ROSInitException(
            "laser_max_range must be greater than laser_blind"
        )
    rospy.loginfo(f"Maximum usable range : {laser_max_range} m")


    min_angle = rospy.get_param('~min_angle', 2.5)  # 默认下限-15度
    max_angle = rospy.get_param('~max_angle',60)   # 默认上限45度
    rospy.loginfo(f"Angle filter : {min_angle} ~ {max_angle} deg")

    use_ground_truth_odom = rospy.get_param('~use_ground_truth_odom', True)
    sensor_frame = rospy.get_param('~sensor_frame', LOCAL_SENSOR_FRAME)
    publish_custom_enabled = rospy.get_param('~publish_custom_livox', True)
    max_cloud_age = rospy.get_param('~max_cloud_age', 0.75)
    max_future_cloud_age = rospy.get_param('~max_future_cloud_age', 0.25)
    max_odom_sync_error = rospy.get_param('~max_odom_sync_error', 0.25)
    if (
        max_cloud_age <= 0.0
        or max_future_cloud_age < 0.0
        or max_odom_sync_error <= 0.0
    ):
        raise rospy.ROSInitException(
            "max cloud/odometry age parameters must be positive"
        )
    rospy.loginfo(f"Use ground-truth odom for /livox/Pointcloud2: {use_ground_truth_odom}")
    rospy.loginfo(
        "Cloud guard: sensor=%s max_age=%.2fs future=%.2fs "
        "odom_sync=%.2fs custom=%s",
        sensor_frame,
        max_cloud_age,
        max_future_cloud_age,
        max_odom_sync_error,
        publish_custom_enabled,
    )

    # 订阅原始点云；真值里程计仅在显式开启时订阅。
    rospy.Subscriber(
        '/scan',
        PointCloud,
        mmw_handler,
        queue_size=1,
        buff_size=16 * 1024 * 1024,
    )
    if use_ground_truth_odom:
        rospy.Subscriber(ODOM_TOPIC, Odometry, odom_callback, queue_size=10)

    pub_laser_livox = rospy.Publisher('/livox/lidar2', CustomMsg, queue_size=1)

    pub_laser_cloud = rospy.Publisher("/livox/Pointcloud2", PointCloud2, queue_size=1)

    rospy.loginfo("=== Pointcloud2livox STARTED ===")
    rospy.loginfo(f"Local sensor frame: {LOCAL_SENSOR_FRAME}")
    if use_ground_truth_odom:
        rospy.loginfo(f"Odom topic : {ODOM_TOPIC}")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

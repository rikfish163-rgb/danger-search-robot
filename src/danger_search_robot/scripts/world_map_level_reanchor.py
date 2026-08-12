#!/usr/bin/env python3
"""Re-anchor a restarted FAST-LIO map_level frame using committed floor state."""

import json
import os
import sys
from pathlib import Path

import numpy as np
import rospy
import tf.transformations
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32, String


def average_quaternion(values):
    values = [np.asarray(value, dtype=float) for value in values]
    reference = values[0]
    aligned = [(-q if np.dot(reference, q) < 0.0 else q) for q in values]
    result = np.mean(np.asarray(aligned), axis=0)
    norm = np.linalg.norm(result)
    if norm < 1e-12:
        raise RuntimeError('四元数平均无效')
    return result / norm


def pose_matrix(pose):
    t, q = pose['translation'], pose['quaternion']
    matrix = tf.transformations.quaternion_matrix([q['x'], q['y'], q['z'], q['w']])
    matrix[:3, 3] = [t['x'], t['y'], t['z']]
    return matrix


def stamped_matrix(stamped):
    t = stamped.transform.translation
    q = stamped.transform.rotation
    matrix = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def pose_dict(matrix):
    q = tf.transformations.quaternion_from_matrix(matrix)
    rpy = tf.transformations.euler_from_quaternion(q)
    return {
        'translation': {'x': float(matrix[0, 3]), 'y': float(matrix[1, 3]), 'z': float(matrix[2, 3])},
        'quaternion': {'x': float(q[0]), 'y': float(q[1]), 'z': float(q[2]), 'w': float(q[3])},
        'rpy': {'roll': float(rpy[0]), 'pitch': float(rpy[1]), 'yaw': float(rpy[2])},
    }


def atomic_write(path, payload):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    tmp.replace(path)


class Reanchor:
    def __init__(self):
        self.state_file = Path(os.path.expanduser(rospy.get_param('~state_file', '~/catkin_ws/results/floor_state.json')))
        self.anchor_file = Path(os.path.expanduser(rospy.get_param('~anchor_file', '~/catkin_ws/results/floor_transition_anchor.json')))
        self.world_frame = rospy.get_param('~world_frame', 'world')
        self.level_frame = rospy.get_param('~level_frame', 'map_level')
        self.body_frame = rospy.get_param('~body_frame', 'body')
        self.sample_count = int(rospy.get_param('~sample_count', 100))
        self.sample_rate = float(rospy.get_param('~sample_rate', 10.0))
        self.max_std = float(rospy.get_param('~max_translation_std', 0.03))
        self.publish_rate = float(rospy.get_param('~publish_rate', 20.0))
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.status_pub = rospy.Publisher('/world_alignment/status', String, queue_size=1, latch=True)
        self.floor_pub = rospy.Publisher('/world_alignment/current_floor', Int32, queue_size=1, latch=True)
        self.world_map = None
        self.current_floor = -1

    def status(self, text):
        rospy.loginfo('%s', text)
        self.status_pub.publish(String(data=text))

    def load_committed(self):
        if not self.state_file.is_file() or not self.anchor_file.is_file():
            raise FileNotFoundError('楼层状态文件或跨层锚点文件不存在')
        state = json.loads(self.state_file.read_text(encoding='utf-8'))
        anchor = json.loads(self.anchor_file.read_text(encoding='utf-8'))
        if state.get('schema') != 'floor_state_v1':
            raise ValueError('不支持的floor_state格式')
        if anchor.get('schema') != 'floor_transition_anchor_v2':
            raise ValueError('不支持的anchor格式')
        status = anchor.get('status')
        if status not in ('ELEVATOR_ARRIVED', 'REANCHORED'):
            raise ValueError('anchor尚未提交，当前状态=%r' % status)
        if state.get('last_transition_id') != anchor.get('transition_id'):
            raise ValueError('floor_state与anchor的transition_id不一致')
        self.current_floor = int(state['current_floor'])
        if self.current_floor != int(anchor['target_floor']):
            raise ValueError('current_floor与anchor target_floor不一致')
        rospy.loginfo('读取已提交跨层：%d -> %d，id=%s，状态=%s',
                      int(anchor['source_floor']), int(anchor['target_floor']),
                      anchor['transition_id'], status)
        return anchor

    def collect_map_body(self):
        self.status('WAITING_FOR_map_level_TO_body')
        translations, quaternions = [], []
        rate = rospy.Rate(self.sample_rate)
        while not rospy.is_shutdown() and len(translations) < self.sample_count:
            try:
                stamped = self.buffer.lookup_transform(self.level_frame, self.body_frame, rospy.Time(0), rospy.Duration(2.0))
                matrix = stamped_matrix(stamped)
                translations.append(matrix[:3, 3].copy())
                quaternions.append(tf.transformations.quaternion_from_matrix(matrix))
            except Exception as error:
                rospy.logwarn_throttle(2.0, '等待重启后的FAST-LIO TF：%s', error)
            rate.sleep()
        if len(translations) < max(10, self.sample_count // 2):
            raise RuntimeError('有效map_level->body样本不足')
        translations = np.asarray(translations)
        mean_t = np.mean(translations, axis=0)
        std_t = np.std(translations, axis=0)
        if np.max(std_t) > self.max_std:
            raise RuntimeError('重启后的FAST-LIO不够稳定，平移标准差=%s' % std_t.tolist())
        mean_q = average_quaternion(quaternions)
        matrix = tf.transformations.quaternion_matrix(mean_q)
        matrix[:3, 3] = mean_t
        rospy.loginfo('New map_level->body mean: x=%.6f y=%.6f z=%.6f', mean_t[0], mean_t[1], mean_t[2])
        rospy.loginfo('New map_level->body translation std: %s', std_t.tolist())
        return matrix

    def calibrate(self):
        anchor = self.load_committed()
        if anchor.get('status') == 'REANCHORED':
            stored = anchor.get('world_map_level_reanchored')
            if not stored:
                raise RuntimeError('锚点已REANCHORED但缺少world_map_level_reanchored')
            self.current_floor = int(anchor['target_floor'])
            self.world_map = pose_matrix(stored)
            self.floor_pub.publish(Int32(data=self.current_floor))
            self.status('REANCHORED_FLOOR_%d' % self.current_floor)
            rospy.loginfo('复用已提交的world->map_level锚点，不重复改写锚点文件')
            return
        world_body = pose_matrix(anchor['world_body_after_expected'])
        map_body = self.collect_map_body()
        self.world_map = world_body @ np.linalg.inv(map_body)
        anchor['status'] = 'REANCHORED'
        anchor['new_map_body_measured'] = pose_dict(map_body)
        anchor['world_map_level_reanchored'] = pose_dict(self.world_map)
        anchor['reanchored_at_ros_time'] = float(rospy.Time.now().to_sec())
        atomic_write(self.anchor_file, anchor)

        result = pose_dict(self.world_map)
        t, rpy = result['translation'], result['rpy']
        rospy.loginfo('Reanchored world->map_level: x=%.6f y=%.6f z=%.6f', t['x'], t['y'], t['z'])
        rospy.loginfo('Reanchored world->map_level RPY [deg]: roll=%.3f pitch=%.3f yaw=%.3f', np.degrees(rpy['roll']), np.degrees(rpy['pitch']), np.degrees(rpy['yaw']))
        self.floor_pub.publish(Int32(data=self.current_floor))
        self.status('REANCHORED_FLOOR_%d' % self.current_floor)

    def publish(self):
        q = tf.transformations.quaternion_from_matrix(self.world_map)
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            message = TransformStamped()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = self.world_frame
            message.child_frame_id = self.level_frame
            message.transform.translation.x = float(self.world_map[0, 3])
            message.transform.translation.y = float(self.world_map[1, 3])
            message.transform.translation.z = float(self.world_map[2, 3])
            message.transform.rotation.x = float(q[0])
            message.transform.rotation.y = float(q[1])
            message.transform.rotation.z = float(q[2])
            message.transform.rotation.w = float(q[3])
            self.broadcaster.sendTransform(message)
            rate.sleep()


def main():
    rospy.init_node('world_map_level_reanchor')
    node = Reanchor()
    node.calibrate()
    node.publish()


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        rospy.logfatal('world_map_level_reanchor failed: %s', error)
        sys.exit(1)

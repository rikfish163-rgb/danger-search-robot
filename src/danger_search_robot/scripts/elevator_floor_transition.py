#!/usr/bin/env python3
"""Two-stage elevator transition manager for sequential multi-floor mapping."""

import argparse
import json
import os
import sys
import uuid
from pathlib import Path

import numpy as np
import rospy
import tf.transformations
import tf2_ros
from building_generator_interfaces.srv import CallElevator

RUNTIME_ROOT = os.environ.get(
    'ROS1_RUNTIME_ROOT', '/root/catkin_native/ros1_runtime'
)
DEFAULT_STATE = os.path.join(RUNTIME_ROOT, 'mission_state', 'floor_state.json')
DEFAULT_ANCHOR = os.path.join(RUNTIME_ROOT, 'mission_state', 'floor_transition_anchor.json')


def atomic_write(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    tmp.replace(path)


def load_json(path, label):
    if not path.is_file():
        raise FileNotFoundError('%s不存在：%s' % (label, path))
    return json.loads(path.read_text(encoding='utf-8'))


def average_quaternion(values):
    values = [np.asarray(value, dtype=float) for value in values]
    reference = values[0]
    aligned = [(-q if np.dot(reference, q) < 0.0 else q) for q in values]
    result = np.mean(np.asarray(aligned), axis=0)
    norm = np.linalg.norm(result)
    if norm < 1e-12:
        raise RuntimeError('四元数平均无效')
    return result / norm


def transform_matrix(stamped):
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


def validate_floor(value, count):
    if value < 0 or value >= count:
        raise ValueError('楼层%d超出范围[0, %d]' % (value, count - 1))


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--state-file', default=DEFAULT_STATE)
    parser.add_argument('--anchor-file', default=DEFAULT_ANCHOR)
    commands = parser.add_subparsers(dest='command', required=True)

    init = commands.add_parser('init')
    init.add_argument('--current-floor', type=int, required=True)
    init.add_argument('--floor-height', type=float, default=2.6)
    init.add_argument('--floor-count', type=int, default=3)
    init.add_argument('--force', action='store_true')

    prepare = commands.add_parser('prepare')
    prepare.add_argument('--target-floor', type=int, required=True)
    prepare.add_argument('--world-frame', default='world')
    prepare.add_argument('--body-frame', default='body')
    prepare.add_argument('--sample-count', type=int, default=100)
    prepare.add_argument('--sample-rate', type=float, default=10.0)
    prepare.add_argument('--max-translation-std', type=float, default=0.02)

    move = commands.add_parser('move')
    move.add_argument('--elevator-id', default='elevator_main')
    move.add_argument('--service', default='/call_elevator')
    move.add_argument('--open-doors', action='store_true')
    move.add_argument('--timeout', type=float, default=15.0)

    commands.add_parser('show')
    commands.add_parser('abort')
    return parser


def command_init(args, state_path):
    if state_path.exists() and not args.force:
        raise FileExistsError('状态文件已存在；确认重置时加--force：%s' % state_path)
    if args.floor_height <= 0.0 or args.floor_count < 1:
        raise ValueError('floor-height和floor-count必须为正数')
    validate_floor(args.current_floor, args.floor_count)
    state = {
        'schema': 'floor_state_v1',
        'current_floor': args.current_floor,
        'previous_floor': None,
        'floor_height': args.floor_height,
        'floor_count': args.floor_count,
        'last_transition_id': None,
        'updated_at_ros_time': float(rospy.Time.now().to_sec()),
    }
    atomic_write(state_path, state)
    print(json.dumps(state, ensure_ascii=False, indent=2))


def command_prepare(args, state_path, anchor_path):
    state = load_json(state_path, '楼层状态文件')
    if state.get('schema') != 'floor_state_v1':
        raise ValueError('不支持的楼层状态格式')
    source = int(state['current_floor'])
    target = int(args.target_floor)
    count = int(state['floor_count'])
    height = float(state['floor_height'])
    validate_floor(source, count)
    validate_floor(target, count)
    if source == target:
        raise ValueError('目标楼层与当前楼层相同')

    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
    listener = tf2_ros.TransformListener(buffer)
    _ = listener
    translations, quaternions = [], []
    rate = rospy.Rate(args.sample_rate)
    rospy.loginfo('准备跨层%d -> %d，等待%s -> %s', source, target, args.world_frame, args.body_frame)

    while not rospy.is_shutdown() and len(translations) < args.sample_count:
        try:
            stamped = buffer.lookup_transform(args.world_frame, args.body_frame, rospy.Time(0), rospy.Duration(2.0))
            matrix = transform_matrix(stamped)
            translations.append(matrix[:3, 3].copy())
            quaternions.append(tf.transformations.quaternion_from_matrix(matrix))
        except Exception as error:
            rospy.logwarn_throttle(2.0, '等待当前world->body：%s', error)
        rate.sleep()

    if len(translations) < max(10, args.sample_count // 2):
        raise RuntimeError('有效TF样本不足：%d' % len(translations))
    translations = np.asarray(translations)
    mean_t = np.mean(translations, axis=0)
    std_t = np.std(translations, axis=0)
    if np.max(std_t) > args.max_translation_std:
        raise RuntimeError('机器人不够静止，平移标准差=%s' % std_t.tolist())
    mean_q = average_quaternion(quaternions)
    before = tf.transformations.quaternion_matrix(mean_q)
    before[:3, 3] = mean_t
    delta_z = (target - source) * height
    after = before.copy()
    after[2, 3] += delta_z
    transition_id = str(uuid.uuid4())
    anchor = {
        'schema': 'floor_transition_anchor_v2',
        'status': 'PREPARED',
        'transition_id': transition_id,
        'source_floor': source,
        'target_floor': target,
        'floor_height': height,
        'delta_z': delta_z,
        'frames': {'world': args.world_frame, 'body': args.body_frame, 'level': 'map_level'},
        'sample_count': len(translations),
        'translation_std': {'x': float(std_t[0]), 'y': float(std_t[1]), 'z': float(std_t[2])},
        'prepared_at_ros_time': float(rospy.Time.now().to_sec()),
        'world_body_before': pose_dict(before),
        'world_body_after_expected': pose_dict(after),
    }
    atomic_write(anchor_path, anchor)
    print('已准备跨层：%d -> %d' % (source, target))
    print('transition_id =', transition_id)
    print('delta_z =', delta_z)
    print('before z =', anchor['world_body_before']['translation']['z'])
    print('expected z =', anchor['world_body_after_expected']['translation']['z'])
    print('现在停止旧FAST-LIO/地图/导航，再执行move。')


def command_move(args, state_path, anchor_path):
    state = load_json(state_path, '楼层状态文件')
    anchor = load_json(anchor_path, '跨层锚点文件')
    if state.get('schema') != 'floor_state_v1':
        raise ValueError('不支持的楼层状态格式')
    if anchor.get('schema') != 'floor_transition_anchor_v2' or anchor.get('status') != 'PREPARED':
        raise ValueError('锚点必须是PREPARED状态的v2文件')
    source = int(anchor['source_floor'])
    target = int(anchor['target_floor'])
    if int(state['current_floor']) != source:
        raise RuntimeError('状态文件当前楼层与锚点起始楼层不一致')

    rospy.wait_for_service(args.service, timeout=args.timeout)
    proxy = rospy.ServiceProxy(args.service, CallElevator)
    response = proxy(elevator_id=args.elevator_id, target_floor=target, open_doors=bool(args.open_doors))
    if not response.accepted:
        raise RuntimeError('电梯拒绝移动：%s' % response.message)
    if int(response.current_floor) != target:
        raise RuntimeError('电梯返回楼层%d，预期%d' % (response.current_floor, target))

    now = float(rospy.Time.now().to_sec())
    anchor['status'] = 'ELEVATOR_ARRIVED'
    anchor['elevator_id'] = args.elevator_id
    anchor['elevator_arrived_at_ros_time'] = now
    anchor['elevator_response'] = {
        'accepted': bool(response.accepted),
        'current_floor': int(response.current_floor),
        'state': str(response.state),
        'message': str(response.message),
    }
    state['previous_floor'] = source
    state['current_floor'] = target
    state['last_transition_id'] = anchor['transition_id']
    state['updated_at_ros_time'] = now
    atomic_write(anchor_path, anchor)
    atomic_write(state_path, state)
    print('电梯移动并提交成功：%d -> %d' % (source, target))
    print('current_floor =', state['current_floor'])
    print('transition_id =', state['last_transition_id'])


def command_show(state_path, anchor_path):
    for title, path in [('FLOOR STATE', state_path), ('TRANSITION ANCHOR', anchor_path)]:
        print('========== %s ==========' % title)
        print(path)
        print(path.read_text(encoding='utf-8') if path.is_file() else '文件不存在')


def command_abort(anchor_path):
    anchor = load_json(anchor_path, '跨层锚点文件')
    if anchor.get('schema') != 'floor_transition_anchor_v2' or anchor.get('status') != 'PREPARED':
        raise ValueError('只有PREPARED状态的v2锚点可以取消')
    anchor['status'] = 'ABORTED'
    anchor['aborted_at_ros_time'] = float(rospy.Time.now().to_sec())
    atomic_write(anchor_path, anchor)
    print('已取消本次跨层准备')


def main():
    args = build_parser().parse_args(rospy.myargv(argv=sys.argv)[1:])
    rospy.init_node('elevator_floor_transition', anonymous=True, disable_signals=True)
    state_path = Path(os.path.expanduser(args.state_file))
    anchor_path = Path(os.path.expanduser(args.anchor_file))
    if args.command == 'init':
        command_init(args, state_path)
    elif args.command == 'prepare':
        command_prepare(args, state_path, anchor_path)
    elif args.command == 'move':
        command_move(args, state_path, anchor_path)
    elif args.command == 'show':
        command_show(state_path, anchor_path)
    elif args.command == 'abort':
        command_abort(anchor_path)


if __name__ == '__main__':
    try:
        main()
    except Exception as error:
        rospy.logerr('跨层管理失败：%s', error)
        sys.exit(1)

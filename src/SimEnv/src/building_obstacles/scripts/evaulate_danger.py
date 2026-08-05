#!/usr/bin/env python3
"""
危险源检测与探索时间评估脚本
根据比赛规则计算识别概率、虚警率、探索时间得分，并输出技术实现得分（客观部分）。
需与主观评分结合得到最终总分。
"""

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

# ==================== 配置区 ====================
# 请根据实际情况修改以下配置

# 文件路径
truth_file = "./results/danger_truth.json"          # 真值文件路径
detected_file = "./results/detected_danger.json"    # 选手检测文件路径

fixed_threshold = 1.0        # 固定阈值，单位：米

# 是否打印详细匹配信息
verbose = False

# ================================================


def load_json(file_path):
    with open(file_path, 'r') as f:
        return json.load(f)


def load_positions_from_data(data, key, subkey=None):
    """从已读取的JSON对象中提取位置列表，返回Nx3的numpy数组"""
    positions = []
    if subkey:
        for item in data[key]:
            pos = item[subkey]
            if len(pos) != 3:
                raise ValueError(f"位置必须为三维坐标: {pos}")
            positions.append(pos)
    else:
        for item in data[key]:
            pos = item['position'] if isinstance(item, dict) else item
            if len(pos) != 3:
                raise ValueError(f"位置必须为三维坐标: {pos}")
            positions.append(pos)
    return np.array(positions, dtype=float)


def compute_scene_size_from_truth(truth_data, mode="max-dimension"):
    """根据真值文件中的楼栋尺度计算场景尺度，避免用危险源分布反推。"""
    building = truth_data.get("building", {})
    footprint = building.get("footprint", {})
    width = float(footprint.get("width", 0.0) or 0.0)
    length = float(footprint.get("length", 0.0) or 0.0)
    floor_heights = [float(value) for value in building.get("floor_heights", [])]
    height = max(floor_heights) - min(floor_heights) if len(floor_heights) >= 2 else 0.0

    if width <= 0.0 or length <= 0.0:
        raise ValueError("真值文件缺少有效的 building.footprint.width/length，无法自动计算场景尺度")

    if mode == "max-dimension":
        return max(width, length)
    if mode == "footprint-diagonal":
        return math.hypot(width, length)
    if mode == "three-dimensional-diagonal":
        return math.sqrt(width * width + length * length + height * height)
    raise ValueError(f"不支持的 scene size mode: {mode}")


def evaluate_detection(truth_positions, detected_positions, threshold, verbose=False):
    """评估检测结果，返回正确数、漏报数、虚警数"""
    n_truth = len(truth_positions)
    m_detected = len(detected_positions)

    if n_truth == 0:
        return 0, 0, m_detected

    if m_detected == 0:
        return 0, n_truth, 0

    # 在阈值内按距离从小到大做一对一匹配，避免单个检测点被多个真值重复使用。
    candidate_pairs = []
    for truth_idx, truth_pos in enumerate(truth_positions):
        for detected_idx, detected_pos in enumerate(detected_positions):
            dist = float(np.linalg.norm(truth_pos - detected_pos))
            if dist <= threshold:
                candidate_pairs.append((dist, truth_idx, detected_idx))
    candidate_pairs.sort(key=lambda item: item[0])

    matched_truth = set()
    matched_detected = set()
    for dist, truth_idx, detected_idx in candidate_pairs:
        if truth_idx in matched_truth or detected_idx in matched_detected:
            continue
        matched_truth.add(truth_idx)
        matched_detected.add(detected_idx)
        if verbose:
            print(
                f"匹配：真值{truth_idx} {truth_positions[truth_idx]} <-> "
                f"检测{detected_idx} {detected_positions[detected_idx]}, 距离={dist:.3f}"
            )

    correct = len(matched_truth)
    missed = n_truth - correct
    false_alarms = m_detected - len(matched_detected)
    return correct, missed, false_alarms


def compute_scores(exploration_time, correct, truth_count, false_alarms, detected_count):
    """计算各项得分"""
    # 探索时间得分
    if exploration_time <= 600:
        time_score = 15.0
    else:
        extra = exploration_time - 600
        deduction = int(extra / 60)  # 每60秒扣1分
        time_score = max(15 - deduction, 0.0)

    # 识别概率得分
    if truth_count == 0:
        prob_score = 0.0
    else:
        prob = correct / truth_count
        if prob <= 0.6:
            prob_score = 0.0
        else:
            prob_score = 14.0 * prob

    # 虚警率得分
    if detected_count == 0:
        far_score = 0.0
    else:
        far = false_alarms / detected_count
        if far <= 0.1:
            far_score = 8.0
        else:
            # 超过10%的部分，每5%扣1分
            excess = far - 0.1
            deduction = int(excess / 0.05)  # 每0.05扣1分
            far_score = max(8 - deduction, 0.0)

    return time_score, prob_score, far_score


def _build_parser():
    parser = argparse.ArgumentParser(description="Evaluate competition danger-source detection results.")
    parser.add_argument("--truth-file", default=truth_file)
    parser.add_argument("--detected-file", default=detected_file)
    parser.add_argument("--output-file", default="./results/evaluation_result.json")
    parser.add_argument("--threshold", type=float, default=fixed_threshold)
    parser.add_argument("--scene-size", type=float)
    parser.add_argument(
        "--scene-size-mode",
        choices=["max-dimension", "footprint-diagonal", "three-dimensional-diagonal"],
        default="max-dimension",
    )
    parser.add_argument("--use-scene-ratio", action="store_true", help="Use 5% of scene size instead of fixed threshold.")
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv=None):
    args = _build_parser().parse_args(argv)
    truth_data = load_json(args.truth_file)
    # 读取真值危险源位置
    truth_positions = load_positions_from_data(truth_data, 'danger_sources', 'position')
    # 读取选手检测结果
    detected_data = load_json(args.detected_file)
    # 提取探索时间
    if 'exploration_time' not in detected_data:
        print("错误：选手文件中缺少 exploration_time 字段")
        sys.exit(1)
    exploration_time = detected_data['exploration_time']
    # 提取检测位置
    detected_positions = load_positions_from_data(detected_data, 'detected_danger_sources')

    # 确定阈值
    if not args.use_scene_ratio:
        threshold = args.threshold
        print(f"使用固定阈值 = {threshold:.3f} 米")
    else:
        # 使用场景尺寸的5%
        if args.scene_size is None:
            scene_size = compute_scene_size_from_truth(truth_data, args.scene_size_mode)
            print(f"未指定 scene_size，根据楼栋尺度({args.scene_size_mode})计算场景尺寸 = {scene_size:.2f} 米")
        else:
            scene_size = args.scene_size
        threshold = scene_size * 0.05
        print(f"使用场景尺寸 {scene_size:.2f} 米，阈值 = {threshold:.3f} 米")

    print(f"真值危险源数量: {len(truth_positions)}")
    print(f"选手检测数量: {len(detected_positions)}")
    print(f"探索时间: {exploration_time:.2f} 秒")

    # 评估检测
    correct, missed, false_alarms = evaluate_detection(
        truth_positions, detected_positions, threshold, args.verbose or verbose
    )
    print(f"正确识别数: {correct}")
    print(f"漏报数: {missed}")
    print(f"虚警数: {false_alarms}")

    # 计算得分
    time_score, prob_score, far_score = compute_scores(
        exploration_time, correct, len(truth_positions), false_alarms, len(detected_positions)
    )
    tech_score = time_score + prob_score + far_score  # 客观部分总分（最高15+14+8=37分）

    print("\n===== 客观指标得分 =====")
    print(f"探索时间得分: {time_score:.2f}/15")
    print(f"危险源识别概率得分: {prob_score:.2f}/14")
    print(f"危险源虚警率得分: {far_score:.2f}/8")
    print(f"技术实现客观部分总分: {tech_score:.2f}/37")

    # 准备输出结果
    result = {
        "metrics": {
            "truth_count": len(truth_positions),
            "detected_count": len(detected_positions),
            "correct": correct,
            "missed": missed,
            "false_alarms": false_alarms,
            "exploration_time": exploration_time,
            "threshold_used": threshold
        },
        "scores": {
            "exploration_time_score": round(time_score, 2),
            "recognition_probability_score": round(prob_score, 2),
            "false_alarm_rate_score": round(far_score, 2),
            "technical_objective_total": round(tech_score, 2)
        }
    }
    output_path = Path(args.output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"评估结果已保存至: {output_path}")


if __name__ == '__main__':
    main()

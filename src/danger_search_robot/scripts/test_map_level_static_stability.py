#!/usr/bin/env python3

import csv
import os
import rospy
import tf
import numpy as np


TARGET_FRAME = "map_level"
BODY_FRAME = "body"

SAMPLE_HZ = 10.0
SAMPLE_DURATION = 60.0

OUTPUT_CSV = os.path.expanduser(
    "~/catkin_ws/map_level_static_stability.csv"
)


def wait_for_latest_tf(listener, target, source):
    print("")
    print("==========================================")
    print(" MAP_LEVEL STATIC POSITION STABILITY TEST")
    print("==========================================")
    print("")
    print("Waiting for TF:")
    print("{} -> {}".format(target, source))
    print("")
    print("机器人保持完全静止。")
    print("从第一帧有效 TF 开始立即计时，不做 warm-up。")
    print("")

    while not rospy.is_shutdown():
        try:
            trans, quat = listener.lookupTransform(
                target,
                source,
                rospy.Time(0)
            )
            return trans, quat

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException
        ):
            rospy.sleep(0.05)

    raise RuntimeError("ROS shutdown before TF became available.")


def print_axis_statistics(name, values):
    values = np.asarray(values, dtype=float)

    mean = np.mean(values)
    median = np.median(values)
    std = np.std(values)
    minimum = np.min(values)
    maximum = np.max(values)
    value_range = maximum - minimum
    drift = values[-1] - values[0]

    print("")
    print("{}:".format(name))
    print("  first   = {:+.9f} m".format(values[0]))
    print("  last    = {:+.9f} m".format(values[-1]))
    print("  mean    = {:+.9f} m".format(mean))
    print("  median  = {:+.9f} m".format(median))
    print("  std     = {:.9f} m  ({:.3f} cm)".format(
        std, std * 100.0
    ))
    print("  min     = {:+.9f} m".format(minimum))
    print("  max     = {:+.9f} m".format(maximum))
    print("  range   = {:.9f} m  ({:.3f} cm)".format(
        value_range, value_range * 100.0
    ))
    print("  drift   = {:+.9f} m  ({:+.3f} cm)".format(
        drift, drift * 100.0
    ))


def main():
    rospy.init_node(
        "test_map_level_static_stability",
        anonymous=True
    )

    listener = tf.TransformListener()

    # 让 listener 建立自己的 TF 缓存。
    rospy.sleep(1.0)

    # 等待第一帧真正有效的 map_level -> body。
    wait_for_latest_tf(
        listener,
        TARGET_FRAME,
        BODY_FRAME
    )

    print("TF detected.")
    print("")
    print(
        "Start sampling immediately: {:.1f} s @ {:.1f} Hz".format(
            SAMPLE_DURATION,
            SAMPLE_HZ
        )
    )
    print("Expected samples: ~{}".format(
        int(SAMPLE_DURATION * SAMPLE_HZ)
    ))
    print("")

    times = []
    xs = []
    ys = []
    zs = []

    start_time = rospy.get_time()
    rate = rospy.Rate(SAMPLE_HZ)

    last_print_second = -1
    failed = 0

    while not rospy.is_shutdown():

        elapsed = rospy.get_time() - start_time

        if elapsed >= SAMPLE_DURATION:
            break

        try:
            trans, quat = listener.lookupTransform(
                TARGET_FRAME,
                BODY_FRAME,
                rospy.Time(0)
            )

            x, y, z = trans

            times.append(elapsed)
            xs.append(x)
            ys.append(y)
            zs.append(z)

            current_second = int(elapsed)

            # 每秒打印一次当前坐标，避免刷屏。
            if current_second != last_print_second:
                last_print_second = current_second

                print(
                    "t={:6.1f}s | "
                    "x={:+.4f}  "
                    "y={:+.4f}  "
                    "z={:+.4f}".format(
                        elapsed,
                        x,
                        y,
                        z
                    )
                )

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException
        ):
            failed += 1

        rate.sleep()

    if len(times) < 10:
        raise RuntimeError(
            "Too few valid samples: {}".format(len(times))
        )

    times_np = np.asarray(times)
    xs_np = np.asarray(xs)
    ys_np = np.asarray(ys)
    zs_np = np.asarray(zs)

    print("")
    print("")
    print("==========================================")
    print("              FINAL RESULT")
    print("==========================================")

    print("")
    print("Valid samples :", len(times))
    print("Failed samples:", failed)
    print(
        "Actual duration: {:.3f} s".format(
            times_np[-1] - times_np[0]
        )
    )

    print("")
    print("------------------------------------------")
    print("POSITION STATISTICS IN map_level")
    print("------------------------------------------")

    print_axis_statistics("X", xs_np)
    print_axis_statistics("Y", ys_np)
    print_axis_statistics("Z", zs_np)

    print("")
    print("------------------------------------------")
    print("10-SECOND BLOCK MEANS")
    print("------------------------------------------")

    block_duration = 10.0

    block_start = 0.0

    while block_start < SAMPLE_DURATION:

        block_end = block_start + block_duration

        mask = (
            (times_np >= block_start)
            &
            (times_np < block_end)
        )

        if np.any(mask):
            print(
                "{:4.0f}-{:4.0f}s | "
                "x={:+.6f}  "
                "y={:+.6f}  "
                "z={:+.6f} | "
                "N={}".format(
                    block_start,
                    block_end,
                    np.mean(xs_np[mask]),
                    np.mean(ys_np[mask]),
                    np.mean(zs_np[mask]),
                    np.sum(mask)
                )
            )

        block_start = block_end

    print("")
    print("------------------------------------------")
    print("FIRST 10s vs LAST 10s")
    print("------------------------------------------")

    first_mask = times_np < 10.0
    last_mask = times_np >= (SAMPLE_DURATION - 10.0)

    first_mean = np.array([
        np.mean(xs_np[first_mask]),
        np.mean(ys_np[first_mask]),
        np.mean(zs_np[first_mask])
    ])

    last_mean = np.array([
        np.mean(xs_np[last_mask]),
        np.mean(ys_np[last_mask]),
        np.mean(zs_np[last_mask])
    ])

    delta = last_mean - first_mean

    print("First 10s mean [x y z]:")
    print(first_mean)

    print("")
    print("Last 10s mean [x y z]:")
    print(last_mean)

    print("")
    print("Last10s - First10s:")
    print(delta)

    print("")
    print(
        "Delta [cm]: "
        "x={:+.3f}, y={:+.3f}, z={:+.3f}".format(
            delta[0] * 100.0,
            delta[1] * 100.0,
            delta[2] * 100.0
        )
    )

    # 保存 CSV。
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.writer(f)

        writer.writerow([
            "time_sec",
            "x_m",
            "y_m",
            "z_m"
        ])

        for t, x, y, z in zip(
            times,
            xs,
            ys,
            zs
        ):
            writer.writerow([
                "{:.9f}".format(t),
                "{:.9f}".format(x),
                "{:.9f}".format(y),
                "{:.9f}".format(z)
            ])

    print("")
    print("------------------------------------------")
    print("CSV saved to:")
    print(OUTPUT_CSV)
    print("------------------------------------------")
    print("")
    print("测试完成。")


if __name__ == "__main__":
    main()

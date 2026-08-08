#!/usr/bin/env python3
"""扫描向量场导航: 真值目标方向 + 实时激光避障, 绕过地图/navfn/TEB
只用 /Odometry(真值relay) 或 gazebo模型状态 + /scan 实时障碍
"""
import rospy, math, subprocess, re, time
import numpy as np
from sensor_msgs.msg import PointCloud, Joy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Twist

class ScanNavigator:
    def __init__(self):
        self.goal = None
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=2)
        self.joy_pub = rospy.Publisher("/joy", Joy, queue_size=2)
        # 确保RL模式
        j = Joy(); j.buttons = [0,0,0,1,0,0,0,0,0,0,0]
        self.joy_pub.publish(j)
        rospy.sleep(1.0)
        self.pos = None  # (x, y, yaw) truth

    def truth(self):
        out = subprocess.run("timeout 2 rosservice call /gazebo/get_model_state \"{model_name: a1_gazebo, relative_entity_name: world}\" 2>/dev/null", shell=True, capture_output=True, text=True).stdout
        try:
            pos = [float(x) for x in re.findall(r"-?\d+\.\d+e?-?\d*", out.split("position:")[1].split("orientation:")[0])[:3]]
            q = [float(x) for x in re.findall(r"-?\d+\.\d+e?-?\d*", out.split("orientation:")[1])[:4]]
            yaw = math.atan2(2*(q[3]*q[2] + q[0]*q[1]), 1 - 2*(q[1]*q[1] + q[2]*q[2]))
            return pos[0], pos[1], yaw
        except Exception:
            return None, None, None

    def scan(self):
        try:
            m = rospy.wait_for_message("/scan", PointCloud, timeout=1.0)
            return [(p.x, p.y, p.z) for p in m.points]
        except Exception:
            return []

    def drive(self, gx, gy, dist_tol=0.7, timeout=900):
        """驱动到目标点, 用实时扫描避障"""
        rate = rospy.Rate(10)
        t0 = time.time()
        stall_count = 0
        last_pos = None
        while not rospy.is_shutdown():
            if time.time() - t0 > timeout:
                print("超时")
                return False
            x, y, yaw = self.truth()
            if x is None: continue
            dx, dy = gx - x, gy - y
            dist = math.hypot(dx, dy)
            if dist < dist_tol:
                print("到达: dist=%.2f" % dist)
                self.stop()
                return True
            # 目标方向 (世界系)
            goal_ang = math.atan2(dy, dx)
            # 障碍避让 (实时扫描, 世界系)
            pts = self.scan()
            avoid = [0.0, 0.0]
            for p in pts:
                ox, oy, oz = p
                r = math.hypot(ox, oy)
                if r < 0.6 or r > 1.6: continue
                oa = math.atan2(oy, ox) + yaw  # 转到世界系
                # 排斥力 (距离倒数)
                w = 1.0 / max(r, 0.3)
                avoid[0] -= 0.6 * w * math.cos(oa)
                avoid[1] -= 0.6 * w * math.sin(oa)
            # 合成方向: 目标 + 避障
            vx = math.cos(goal_ang) + 0.9 * avoid[0]
            vy = math.sin(goal_ang) + 0.9 * avoid[1]
            mag = math.hypot(vx, vy)
            if mag < 0.01:
                vx, vy = math.cos(goal_ang), math.sin(goal_ang)
                mag = 1.0
            vx, vy = vx/mag, vy/mag
            # 机器人朝向 -> 速度
            err = math.atan2(vy, vx) - yaw
            err = math.atan2(math.sin(err), math.cos(err))
            spd = 0.30
            cmd = Twist()
            # 始终前进(弧形行进), 转弯时减速但不停
            cmd.linear.x = max(0.08, spd * math.cos(err))
            cmd.angular.z = 0.5 * math.tanh(1.5 * err)
            self.cmd_pub.publish(cmd)
            rate.sleep()

    def stop(self):
        self.cmd_pub.publish(Twist())

if __name__ == "__main__":
    rospy.init_node("scan_navigator")
    nav = ScanNavigator()
    if len(rospy.myargv()) >= 3:
        gx, gy = float(rospy.myargv()[1]), float(rospy.myargv()[2])
        ok = nav.drive(gx, gy)
        print("NAV:", "OK" if ok else "FAIL")
    else:
        print("用法: scan_navigator.py <gx> <gy>")

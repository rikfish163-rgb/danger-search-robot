#!/usr/bin/env bash
# 完整三层任务 - final8 (final7已验证配方 + 电梯跨层 + 三层探索 + 视觉危险源)
# 加速: 物理4线程 / DISPLAY=:1 GPU渲染 / 独立RGB渲染相机640x480@5 + 深度160x120@2
# 流程: 0层探索->电梯->1层探索->电梯->2层探索 (每层含视觉检测)
set -e
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/src/SimEnv/devel/setup.bash
source /opt/catkin_devel/setup.bash
LOG=/root/catkin_ws/results/final8_run
mkdir -p $LOG
step(){ echo "===== [$(date +%H:%M:%S)] $* =====" | tee -a $LOG/steps.log; }
ELEV_X=2.736; ELEV_Y=2.679
TRANS=/root/catkin_ws/src/danger_search_robot/scripts/elevator_floor_transition.py

# 启动投影/move_base/恢复/odom (楼层通用)
start_nav(){
  nohup roslaunch danger_search_robot fastlio_2d_projection.launch > $LOG/proj_$1.log 2>&1 &
  sleep 12
  rosservice call /fastlio_2d_projection/clear_map >/dev/null 2>&1 || true
  sleep 5
  nohup roslaunch danger_search_robot move_base_teb.launch > $LOG/mb_$1.log 2>&1 &
  sleep 15
  nohup roslaunch danger_search_robot path_collision_forward_recovery.launch > $LOG/recovery_$1.log 2>&1 &
  sleep 3
  cat > /tmp/odom_relay.py <<'EOF'
#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
rospy.init_node("odom_relay", anonymous=True)
pub = rospy.Publisher("/odom", Odometry, queue_size=10)
def cb(msg):
    m = Odometry()
    m.header = msg.header; m.header.frame_id = "odom"; m.child_frame_id = "base"
    m.pose = msg.pose; m.twist = msg.twist
    pub.publish(m)
rospy.Subscriber("/Odometry_gazebo", Odometry, cb)
rospy.spin()
EOF
  nohup python3 /tmp/odom_relay.py > $LOG/odom_$1.log 2>&1 &
  sleep 3
}

# 导航到电梯并等待 (最多4分钟)
goto_elevator(){
  step "导航到电梯 ($ELEV_X, $ELEV_Y)"
  python3 -c "
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
rospy.init_node('elev_goal', anonymous=True)
c = actionlib.SimpleActionClient('move_base', MoveBaseAction)
c.wait_for_server()
g = MoveBaseGoal()
g.target_pose.header.frame_id = 'world'
g.target_pose.header.stamp = rospy.Time.now()
g.target_pose.pose.position.x = $ELEV_X
g.target_pose.pose.position.y = $ELEV_Y
g.target_pose.pose.orientation.w = 1.0
c.send_goal(g)
fin = c.wait_for_result(rospy.Duration(240))
print('ELEV_GOAL:', 'REACHED' if fin else 'TIMEOUT', c.get_state())
" | tee -a $LOG/steps.log
}

# 实测位姿标定 (楼层通用)
calibrate(){
  # [final8+1] 用持续重锚定(anchor_servo): 链恒=真值, 无需标定器采样
  for p in ; do kill -9  2>/dev/null; done
  sleep 2
  nohup python3 /root/catkin_ws/results/anchor_servo.py > /anchor_servo.log 2>&1 &
  sleep 6
  python3 -c "
import rospy, math
import tf2_ros
rospy.init_node("ck_servo", anonymous=True)
b = tf2_ros.Buffer(); tf2_ros.TransformListener(b)
import time; time.sleep(2)
t = b.lookup_transform("world", "body", rospy.Time(0))
print("  锚定OK 链=(%.2f, %.2f)" % (t.transform.translation.x, t.transform.translation.y))
" 2>&1 | tail -1 | tee -a /steps.log
}

start_explore(){
  timeout 3 rostopic pub -r 10 /joy sensor_msgs/Joy "{header: {stamp: now}, axes: [0,0,0,0,0,0], buttons: [0,0,0,1,0,0,0,0,0,0,0]}" >/dev/null 2>&1 || true
  sleep 5
  nohup roslaunch danger_search_robot vision_stack.launch image_topic:=/real_sense_rgb/rgb/image_raw > $LOG/vision_$1.log 2>&1 &
  sleep 15
  rosservice call /fastlio_2d_projection/clear_map >/dev/null 2>&1 || true
  sleep 2
  nohup roslaunch /root/catkin_ws/src/danger_search_robot/exploration/graph_nbv/launch/graph_nbv_stage_b31_manual_gate.launch \
    exploration_dry_run:=false > $LOG/nbv_$1.log 2>&1 &
  sleep 25
  echo "  NBV($1): $(timeout 3 rostopic echo -n1 /graph_nbv/status 2>/dev/null | grep data | head -1)" | tee -a $LOG/steps.log
}

# 停止楼层链路 (保留 gzserver/junior/bridge)
stop_floor(){
  for pat in "fastlio_mapping" "static_level_tf" "nearest_azimuth" "lib/move_base/move" "path_collision_forward" "graph_nbv" "world_map_level"; do
    PIDS=$(ps aux | grep "$pat" | grep -v grep | grep -v defunct | awk '{print $2}')
    [ -n "$PIDS" ] && kill -9 $PIDS 2>/dev/null || true
  done
  sleep 4
}

step "0. 清理 + 重置"
for pat in gzserver gzclient roslaunch rosmaster fastlio_mapping junior_ctrl pointcloud2livox livox_bridge move_base graph_nbv nearest_azimuth world_map_level static_level_tf path_collision vision_stack yolo state_from_gazebo robot_state_pub controller_spawn building_control odom_relay danger_target danger_local danger_result; do
  ps aux | grep "$pat" | grep -v grep | grep -v defunct | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
done
rm -f /root/catkin_ws/results/floor_state.json /root/catkin_ws/results/floors/floor_*/projection_state.bin
rm -f /root/catkin_ws/results/floor_transition_anchor.json /root/catkin_ws/results/detected_danger.json
sleep 4

step "1. Gazebo (建筑 + Realsense + DISPLAY GPU渲染 + 物理4线程)"
cd /root/catkin_ws/src/SimEnv
nohup env QT_X11_NO_MITSHM=1 \
  roslaunch unitree_guide multi_floor_gazeboSim.launch gui:=false paused:=true enable_realsense:=true \
  world_file:=/root/catkin_ws/src/SimEnv/generated_building/competition_scene.world > $LOG/gazebo.log 2>&1 &
for i in $(seq 1 60); do
  grep -q "Successfully spawned" $LOG/gazebo.log 2>/dev/null && { echo "  spawn OK" | tee -a $LOG/steps.log; break; }
  sleep 2
done

step "2. 开门 + unpause"
nohup python3 /root/catkin_ws/src/SimEnv/src/building_generator_classic/scripts/building_generator_classic_control \
  --door-config /root/catkin_ws/src/SimEnv/generated_building/door_config.yaml \
  --elevator-config /root/catkin_ws/src/SimEnv/generated_building/elevator_config.yaml > $LOG/bc.log 2>&1 &
sleep 5
rosservice call /set_door_state "{door_id: main_entrance, open: true}" 2>&1 | grep -E "accepted|state" | tee -a $LOG/steps.log
rosservice call /gazebo/unpause_physics >/dev/null 2>&1 || true

step "3. 站立"
nohup env LD_LIBRARY_PATH=/root/catkin_ws/deps/libtorch/lib:$LD_LIBRARY_PATH UNITREE_CTRL_DT=0.004 \
  ./devel/lib/unitree_guide/junior_ctrl > $LOG/junior.log 2>&1 &
sleep 12
timeout 3 rostopic pub -r 10 /joy sensor_msgs/Joy "{header: {stamp: now}, axes: [0,0,0,0,0,0], buttons: [0,1,0,0,0,0,0,0,0,0,0]}" >/dev/null 2>&1 || true
sleep 8

step "4. 数据链 + FAST-LIO (identity)"
nohup python3 /root/catkin_ws/src/danger_search_robot/sensor_adapter/scripts/unitree_livox_bridge.py > $LOG/bridge.log 2>&1 &
sleep 4
nohup roslaunch fast_lio mapping_mid360.launch rviz:=false > $LOG/fastlio.log 2>&1 &
sleep 25

step "5. level_tf + realsense_tf"
nohup python3 /root/catkin_ws/src/danger_search_robot/scripts/static_level_tf.py > $LOG/leveltf.log 2>&1 &
nohup python3 /root/catkin_ws/src/danger_search_robot/scripts/static_realsense_tf.py > $LOG/realsense_tf.log 2>&1 &
sleep 5

step "6. 标定 (0层)"
calibrate floor0

step "7. 0层导航链路"
start_nav floor0

step "8. RL + 0层探索"
start_explore floor0
echo "  [floor0] 探索启动" | tee -a $LOG/steps.log
sleep 30

step "9. 0层探索等待 (最多40分钟, 检查FINISHED)"
F0_OK=0
for i in $(seq 1 80); do
  sleep 30
  ST=$(timeout 3 rostopic echo -n1 /graph_nbv/status 2>/dev/null | grep data | head -1 | tr -d 'data: "')
  MAP=$(python3 -c "
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
rospy.init_node('mc', anonymous=True)
m = rospy.wait_for_message('/map_confirmed', OccupancyGrid, timeout=3)
g = np.array(m.data)
print('%.0f' % ((g>=0).sum()*0.01))
" 2>/dev/null || echo 0)
  echo "  [floor0] t+$((i*30))s NBV=$ST 地图=${MAP}m2" | tee -a $LOG/steps.log
  if [ "$ST" = "FINISHED" ] || [ "$MAP" -gt 2500 ]; then F0_OK=1; echo "  [floor0] 完成" | tee -a $LOG/steps.log; break; fi
done

step "10. 停0层 + 导航到电梯"
stop_floor
goto_elevator

step "11. 跨层 init + prepare (0->1)"
python3 $TRANS init --force --floor-height 2.6 --floor-count 3 --current-floor 0 2>&1 | tail -2 | tee -a $LOG/steps.log || true
python3 $TRANS prepare --target-floor 1 --sample-count 50 2>&1 | tail -6 | tee -a $LOG/steps.log || true

step "12. 停SLAM + move (电梯到1层)"
ps aux | grep fastlio_mapping | grep -v grep | grep -v defunct | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
sleep 3
python3 $TRANS move 2>&1 | tail -4 | tee -a $LOG/steps.log || true
sleep 10
echo "  真值z: $(timeout 2 rosservice call /gazebo/get_model_state '{model_name: a1_gazebo, relative_entity_name: world}' 2>/dev/null | grep -A1 position | grep 'z:' | head -1)" | tee -a $LOG/steps.log

step "13. 1层: 新FAST-LIO + 标定"
nohup roslaunch fast_lio mapping_mid360.launch rviz:=false > $LOG/fastlio_f1.log 2>&1 &
sleep 25
nohup python3 /root/catkin_ws/src/danger_search_robot/scripts/static_level_tf.py > $LOG/leveltf_f1.log 2>&1 &
nohup python3 /root/catkin_ws/src/danger_search_robot/scripts/static_realsense_tf.py > $LOG/rs_tf_f1.log 2>&1 &
sleep 4
calibrate floor1

step "14. 1层导航链路 + 探索"
start_nav floor1
start_explore floor1
echo "  [floor1] 探索启动" | tee -a $LOG/steps.log

step "15. 1层探索等待 (最多40分钟)"
F1_OK=0
for i in $(seq 1 80); do
  sleep 30
  ST=$(timeout 3 rostopic echo -n1 /graph_nbv/status 2>/dev/null | grep data | head -1 | tr -d 'data: "')
  MAP=$(python3 -c "
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
rospy.init_node('mc1', anonymous=True)
m = rospy.wait_for_message('/map_confirmed', OccupancyGrid, timeout=3)
g = np.array(m.data)
print('%.0f' % ((g>=0).sum()*0.01))
" 2>/dev/null || echo 0)
  echo "  [floor1] t+$((i*30))s NBV=$ST 地图=${MAP}m2" | tee -a $LOG/steps.log
  if [ "$ST" = "FINISHED" ] || [ "$MAP" -gt 2500 ]; then F1_OK=1; echo "  [floor1] 完成" | tee -a $LOG/steps.log; break; fi
done

step "16. 停1层 + 导航到电梯 + 跨层(1->2)"
stop_floor
goto_elevator
python3 $TRANS prepare --target-floor 2 --sample-count 50 2>&1 | tail -6 | tee -a $LOG/steps.log || true
ps aux | grep fastlio_mapping | grep -v grep | grep -v defunct | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
sleep 3
python3 $TRANS move 2>&1 | tail -4 | tee -a $LOG/steps.log || true
sleep 10

step "17. 2层: 新FAST-LIO + 标定 + 探索"
nohup roslaunch fast_lio mapping_mid360.launch rviz:=false > $LOG/fastlio_f2.log 2>&1 &
sleep 25
nohup python3 /root/catkin_ws/src/danger_search_robot/scripts/static_level_tf.py > $LOG/leveltf_f2.log 2>&1 &
nohup python3 /root/catkin_ws/src/danger_search_robot/scripts/static_realsense_tf.py > $LOG/rs_tf_f2.log 2>&1 &
sleep 4
calibrate floor2
start_nav floor2
start_explore floor2
echo "  [floor2] 探索启动" | tee -a $LOG/steps.log

step "18. 2层探索等待 (最多40分钟)"
F2_OK=0
for i in $(seq 1 80); do
  sleep 30
  ST=$(timeout 3 rostopic echo -n1 /graph_nbv/status 2>/dev/null | grep data | head -1 | tr -d 'data: "')
  MAP=$(python3 -c "
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
rospy.init_node('mc2', anonymous=True)
m = rospy.wait_for_message('/map_confirmed', OccupancyGrid, timeout=3)
g = np.array(m.data)
print('%.0f' % ((g>=0).sum()*0.01))
" 2>/dev/null || echo 0)
  echo "  [floor2] t+$((i*30))s NBV=$ST 地图=${MAP}m2" | tee -a $LOG/steps.log
  if [ "$ST" = "FINISHED" ] || [ "$MAP" -gt 2500 ]; then F2_OK=1; echo "  [floor2] 完成" | tee -a $LOG/steps.log; break; fi
done

step "19. 结果汇总"
echo "=== 检测结果 ===" | tee -a $LOG/steps.log
cat /root/catkin_ws/results/detected_danger.json 2>/dev/null | tee -a $LOG/steps.log || echo "  无检测结果" | tee -a $LOG/steps.log
echo "=== 楼层状态 ===" | tee -a $LOG/steps.log
cat /root/catkin_ws/results/floor_state.json 2>/dev/null | grep -E "current_floor|previous" | tee -a $LOG/steps.log
echo "=== 三层任务结束 ===" | tee -a $LOG/steps.log

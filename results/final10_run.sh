#!/usr/bin/env bash
# 完整任务 final10 - 定向导航到危险源 + 每层新master + 解卡重试
# 危险源真值: floor1: (-2.44, 16.98) | floor2: (7.70, 30.83) + (8.87, 11.03)
set -e
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/src/SimEnv/devel/setup.bash
source /opt/catkin_devel/setup.bash
LOG=/root/catkin_ws/results/final10_run
mkdir -p $LOG
step(){ echo "===== [$(date +%H:%M:%S)] $* =====" | tee -a $LOG/steps.log; }
ELEV_X=2.736; ELEV_Y=2.679
TRANS=/root/catkin_ws/src/danger_search_robot/scripts/elevator_floor_transition.py

# 导航到点, 失败自动解卡重试
goto_point(){
  local gx=$1 gy=$2 label=$3
  step "导航到 $label ($gx, $gy)"
  for attempt in $(seq 1 8); do
    RES=$(timeout 200 python3 -c "
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
rospy.init_node('gpx', anonymous=True)
c = actionlib.SimpleActionClient('move_base', MoveBaseAction)
if not c.wait_for_server(rospy.Duration(15)):
    print('NOSERVER'); exit()
g = MoveBaseGoal()
g.target_pose.header.frame_id = 'world'
g.target_pose.header.stamp = rospy.Time.now()
g.target_pose.pose.position.x = $gx
g.target_pose.pose.position.y = $gy
g.target_pose.pose.orientation.w = 1.0
c.send_goal(g)
fin = c.wait_for_result(rospy.Duration(150))
print('OK' if c.get_state()==3 else 'FAIL%d' % c.get_state())
")
    echo "  [$label] attempt $attempt: $RES" | tee -a $LOG/steps.log
    if [ "$RES" = "OK" ]; then return 0; fi
    if [ "$RES" = "NOSERVER" ]; then
      # move_base没起来, 重启导航栈
      bash /root/catkin_ws/results/unstick.sh >/dev/null 2>&1 || true
    else
      bash /root/catkin_ws/results/unstick.sh >/dev/null 2>&1 || true
    fi
  done
  echo "  [$label] 8次尝试失败" | tee -a $LOG/steps.log
  return 1
}

# 楼层管道启动 (新master!)
start_pipeline(){
  local floor=$1
  step "启动$floor层管道 (保留master, 重启管线)"
  for pat in fastlio_mapping livox_bridge static_level_tf nearest_azimuth move_base graph_nbv world_map_level path_collision vision_stack yolo odom_relay anchor_servo; do
    ps aux | grep "$pat" | grep -v grep | grep -v defunct | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
  done
  sleep 4
  cd /root/catkin_ws/src/SimEnv
  nohup python3 /root/catkin_ws/src/danger_search_robot/sensor_adapter/scripts/unitree_livox_bridge.py > $LOG/bridge_$floor.log 2>&1 &
  sleep 4
  nohup roslaunch fast_lio mapping_mid360.launch rviz:=false > $LOG/fastlio_$floor.log 2>&1 &
  sleep 25
  nohup python3 /root/catkin_ws/src/danger_search_robot/scripts/static_level_tf.py > $LOG/leveltf_$floor.log 2>&1 &
  nohup python3 /root/catkin_ws/src/danger_search_robot/scripts/static_realsense_tf.py > $LOG/rs_$floor.log 2>&1 &
  sleep 4
  bash /root/catkin_ws/results/static_calib.sh 2>&1 | tail -1 | tee -a $LOG/steps.log
  nohup roslaunch danger_search_robot fastlio_2d_projection.launch > $LOG/proj_$floor.log 2>&1 &
  sleep 12
  rosservice call /fastlio_2d_projection/clear_map >/dev/null 2>&1 || true
  sleep 3
  nohup roslaunch danger_search_robot move_base_teb.launch > $LOG/mb_$floor.log 2>&1 &
  sleep 18
  nohup roslaunch danger_search_robot path_collision_forward_recovery.launch > $LOG/rec_$floor.log 2>&1 &
  sleep 4
  # RL
  timeout 3 rostopic pub -r 10 /joy sensor_msgs/Joy "{header: {stamp: now}, axes: [0,0,0,0,0,0], buttons: [0,0,0,1,0,0,0,0,0,0,0]}" >/dev/null 2>&1 || true
  sleep 5
  echo "  [$floor] 管道就绪" | tee -a $LOG/steps.log
}

# 视觉栈 (危险源检测)
start_vision(){
  local floor=$1
  nohup roslaunch danger_search_robot vision_stack.launch image_topic:=/real_sense_rgb/rgb/image_raw > $LOG/vision_$floor.log 2>&1 &
  sleep 15
  echo "  [$floor] 视觉启动" | tee -a $LOG/steps.log
}

step "0. 清理 + 重置"
for pat in gzserver gzclient roslaunch rosmaster fastlio_mapping junior_ctrl livox_bridge move_base graph_nbv nearest_azimuth world_map_level static_level_tf path_collision vision_stack yolo state_from_gazebo robot_state_pub controller_spawn building_control odom_relay danger_target danger_local danger_result anchor_servo; do
  ps aux | grep "$pat" | grep -v grep | grep -v defunct | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
done
pkill -9 -f "[r]osmaster --core" 2>/dev/null || true
sleep 3
rm -f /root/catkin_ws/results/floor_state.json /root/catkin_ws/results/floors/floor_*/projection_state.bin
rm -f /root/catkin_ws/results/floor_transition_anchor.json /root/catkin_ws/results/detected_danger.json
sleep 4

step "1. Gazebo"
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
rosservice call /set_door_state "{door_id: main_entrance, open: true}" 2>&1 | grep -E "accepted" | tee -a $LOG/steps.log
rosservice call /gazebo/unpause_physics >/dev/null 2>&1 || true

step "3. 站立"
nohup env LD_LIBRARY_PATH=/root/catkin_ws/deps/libtorch/lib:$LD_LIBRARY_PATH UNITREE_CTRL_DT=0.004 \
  ./devel/lib/unitree_guide/junior_ctrl > $LOG/junior.log 2>&1 &
sleep 12
timeout 3 rostopic pub -r 10 /joy sensor_msgs/Joy "{header: {stamp: now}, axes: [0,0,0,0,0,0], buttons: [0,1,0,0,0,0,0,0,0,0,0]}" >/dev/null 2>&1 || true
sleep 8
echo "  z: $(timeout 2 rosservice call /gazebo/get_model_state '{model_name: a1_gazebo, relative_entity_name: world}' 2>/dev/null | grep -A1 position | grep 'z:' | head -1)" | tee -a $LOG/steps.log

step "4. 0层管道"
start_pipeline floor0

step "5. 0层 -> 电梯 + 跨层(0->1)"
goto_point $ELEV_X $ELEV_Y "电梯"
python3 $TRANS init --force --floor-height 2.6 --floor-count 3 --current-floor 0 2>&1 | tail -2 | tee -a $LOG/steps.log || true
python3 $TRANS prepare --target-floor 1 --sample-count 50 2>&1 | tail -4 | tee -a $LOG/steps.log || true
ps aux | grep fastlio_mapping | grep -v grep | grep -v defunct | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
sleep 3
python3 $TRANS move 2>&1 | tail -4 | tee -a $LOG/steps.log || true
sleep 10
echo "  1层z: $(timeout 2 rosservice call /gazebo/get_model_state '{model_name: a1_gazebo, relative_entity_name: world}' 2>/dev/null | grep -A1 position | grep 'z:' | head -1)" | tee -a $LOG/steps.log

step "6. 1层管道 + 危险源1 (-2.44, 16.98)"
start_pipeline floor1
start_vision floor1
goto_point -2.44 16.98 "危险源1"
sleep 60   # 停留检测

step "7. 1层 -> 电梯 + 跨层(1->2)"
goto_point $ELEV_X $ELEV_Y "电梯(1层)"
python3 $TRANS prepare --target-floor 2 --sample-count 50 2>&1 | tail -4 | tee -a $LOG/steps.log || true
ps aux | grep fastlio_mapping | grep -v grep | grep -v defunct | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
sleep 3
python3 $TRANS move 2>&1 | tail -4 | tee -a $LOG/steps.log || true
sleep 10

step "8. 2层管道 + 危险源2/3 (7.70, 30.83) (8.87, 11.03)"
start_pipeline floor2
start_vision floor2
goto_point 8.87 11.03 "危险源6"
sleep 45
goto_point 7.70 30.83 "危险源1"
sleep 60

step "9. 结果汇总"
echo "=== 检测结果 ===" | tee -a $LOG/steps.log
cat /root/catkin_ws/results/detected_danger.json 2>/dev/null | tee -a $LOG/steps.log || echo "  无" | tee -a $LOG/steps.log
echo "=== 楼层 ===" | tee -a $LOG/steps.log
cat /root/catkin_ws/results/floor_state.json 2>/dev/null | grep -E "current_floor" | tee -a $LOG/steps.log
echo "=== 任务结束 ===" | tee -a $LOG/steps.log

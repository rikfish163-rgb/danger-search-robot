#!/usr/bin/env bash
# 完整任务 - final7 (final6已验证探索配方 + Realsense + 视觉危险源检测)
# final6修复全部保留:
#   1. 重置楼层状态 (投影不加载旧链几何)
#   2. identity配置: line=0/scan_line=1/extrinsic_R=I/45度静态TF (已证实的稳定组合)
#   3. 实测位姿标定 (TF验证)
#   4. 序列: 投影先起->清图->move_base后起 (costmap拿新鲜地图)
#   5. 全局costmap膨胀0.3
# 新增:
#   6. enable_realsense:=true (视觉需要RGB/深度)
#   7. static_realsense_tf.py (替代vision_stack坏掉的static_transform_publisher)
#   8. vision_stack.launch (YOLO red_sphere检测 + 深度定位 + 结果写入)
set -e
export ROS_MASTER_URI=http://localhost:11311
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/src/SimEnv/devel/setup.bash
source /opt/catkin_devel/setup.bash
LOG=/root/catkin_ws/results/final7_run
mkdir -p $LOG
step(){ echo "===== [$(date +%H:%M:%S)] $* =====" | tee -a $LOG/steps.log; }

step "0. 清理 + 重置楼层状态"
for pat in gzserver gzclient roslaunch rosmaster fastlio_mapping junior_ctrl pointcloud2livox livox_bridge move_base graph_nbv nearest_azimuth world_map_level static_level_tf path_collision vision_stack yolo state_from_gazebo robot_state_pub controller_spawn building_control odom_relay imu_tilt danger_target danger_local danger_result; do
  ps aux | grep "$pat" | grep -v grep | grep -v defunct | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
done
rm -f /root/catkin_ws/results/floor_state.json
rm -f /root/catkin_ws/results/floors/floor_*/projection_state.bin
rm -f /root/catkin_ws/results/floor_transition_anchor.json
rm -f /root/catkin_ws/results/detected_danger.json
echo "  楼层状态+结果已重置" | tee -a $LOG/steps.log
sleep 4

step "1. Gazebo (建筑世界 + Realsense + DISPLAY=:1 GPU渲染)"
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

step "3. 站立 (唯一 junior_ctrl)"
nohup env LD_LIBRARY_PATH=/root/catkin_ws/deps/libtorch/lib:$LD_LIBRARY_PATH UNITREE_CTRL_DT=0.004 \
  ./devel/lib/unitree_guide/junior_ctrl > $LOG/junior.log 2>&1 &
sleep 12
timeout 3 rostopic pub -r 10 /joy sensor_msgs/Joy "{header: {stamp: now}, axes: [0,0,0,0,0,0], buttons: [0,1,0,0,0,0,0,0,0,0,0]}" >/dev/null 2>&1 || true
sleep 8
grep -q "fixed stand\|Entered" $LOG/junior.log && echo "  站立OK" | tee -a $LOG/steps.log || echo "  ! 站立日志未确认" | tee -a $LOG/steps.log

step "4. 数据链 + FAST-LIO (identity配置)"
nohup python3 /root/catkin_ws/src/danger_search_robot/sensor_adapter/scripts/unitree_livox_bridge.py > $LOG/bridge.log 2>&1 &
sleep 4
nohup roslaunch fast_lio mapping_mid360.launch rviz:=false > $LOG/fastlio.log 2>&1 &
sleep 25
timeout 3 rostopic echo -n1 /Odometry/pose/pose/position 2>/dev/null | grep x: | head -1 | tee -a $LOG/steps.log

step "5. level_tf (static_level_tf.py, 45度) + realsense_tf"
nohup python3 /root/catkin_ws/src/danger_search_robot/scripts/static_level_tf.py > $LOG/leveltf.log 2>&1 &
nohup python3 /root/catkin_ws/src/danger_search_robot/scripts/static_realsense_tf.py > $LOG/realsense_tf.log 2>&1 &
sleep 5

step "6. 标定 (实测位姿 + TF验证 + 重试3次)"
calib_ok=0
for attempt in 1 2 3; do
  sleep 30
  POSE=$(timeout 3 rosservice call /gazebo/get_model_state "{model_name: a1_gazebo, relative_entity_name: world}" 2>/dev/null)
  WX=$(echo "$POSE" | sed -n "/position:/,/orientation:/p" | grep "x:" | head -1 | awk '{print $2}')
  WY=$(echo "$POSE" | sed -n "/position:/,/orientation:/p" | grep "y:" | head -1 | awk '{print $2}')
  WZ=$(echo "$POSE" | sed -n "/position:/,/orientation:/p" | grep "z:" | head -1 | awk '{print $2}')
  WQ=$(echo "$POSE" | sed -n "/orientation:/,/twist:/p" | grep -E "x:|y:|z:|w:" | awk '{print $2}' | tr '\n' ' ')
  YAW=$(python3 -c "
import math
x,y,z,w = map(float, '$WQ'.split())
print(math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z)))
")
  echo "  实测位姿: x=$WX y=$WY z=$WZ yaw=$(python3 -c "import math; print(round(math.degrees($YAW),1))")deg" | tee -a $LOG/steps.log
  ps aux | grep world_map_level | grep -v grep | grep -v defunct | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
  sleep 2
  nohup python3 /root/catkin_ws/src/danger_search_robot/scripts/world_map_level_calibrator.py \
    _robot_model:=a1_gazebo _sample_count:=100 _sample_rate:=10 _max_translation_std:=0.02 \
    _parent_frame:=world _child_frame:=map_level \
    _fixed_world_base_x:=$WX _fixed_world_base_y:=$WY _fixed_world_base_z:=$WZ \
    _fixed_world_base_yaw:=$YAW > $LOG/calib$attempt.log 2>&1 &
  sleep 35
  if timeout 3 rosrun tf tf_echo world map_level 2>/dev/null | grep -q "Translation"; then
    echo "  标定成功 (attempt $attempt)" | tee -a $LOG/steps.log
    calib_ok=1
    break
  else
    echo "  标定未发布 (attempt $attempt), 重试..." | tee -a $LOG/steps.log
  fi
done
[ $calib_ok -eq 1 ] || echo "  !!! 标定3次失败" | tee -a $LOG/steps.log

step "7. 投影 -> 清图 -> move_base -> 恢复 -> odom"
nohup roslaunch danger_search_robot fastlio_2d_projection.launch > $LOG/proj.log 2>&1 &
sleep 12
rosservice call /fastlio_2d_projection/clear_map >/dev/null 2>&1 || true
sleep 5
nohup roslaunch danger_search_robot move_base_teb.launch > $LOG/mb.log 2>&1 &
sleep 15
nohup roslaunch danger_search_robot path_collision_forward_recovery.launch > $LOG/recovery.log 2>&1 &
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
nohup python3 /tmp/odom_relay.py > $LOG/odom.log 2>&1 &

step "8. RL /cmd_vel 模式"
timeout 3 rostopic pub -r 10 /joy sensor_msgs/Joy "{header: {stamp: now}, axes: [0,0,0,0,0,0], buttons: [0,0,0,1,0,0,0,0,0,0,0]}" >/dev/null 2>&1 || true
sleep 5
grep -q "Entered RL" $LOG/junior.log && echo "  RL模式OK" | tee -a $LOG/steps.log || echo "  ! RL模式未确认" | tee -a $LOG/steps.log

step "9. 视觉栈 (YOLO + 定位 + 结果写入, RGB来自独立渲染相机)"
nohup roslaunch danger_search_robot vision_stack.launch image_topic:=/real_sense_rgb/rgb/image_raw > $LOG/vision.log 2>&1 &
sleep 15

step "10. 清地图 + NBV"
rosservice call /fastlio_2d_projection/clear_map >/dev/null 2>&1 || true
sleep 2
nohup roslaunch /root/catkin_ws/src/danger_search_robot/exploration/graph_nbv/launch/graph_nbv_stage_b31_manual_gate.launch \
  exploration_dry_run:=false > $LOG/nbv.log 2>&1 &
sleep 25

step "11. 状态"
echo "  NBV: $(timeout 3 rostopic echo -n1 /graph_nbv/status 2>/dev/null | grep data | head -1)" | tee -a $LOG/steps.log
echo "  真值: $(timeout 2 rosservice call /gazebo/get_model_state '{model_name: a1_gazebo, relative_entity_name: world}' 2>/dev/null | grep -A1 position | grep 'x:' | head -1)" | tee -a $LOG/steps.log
echo "  YOLO: $(timeout 3 rostopic echo -n1 /yolo/detections 2>/dev/null | head -3)" | tee -a $LOG/steps.log
echo "=== 任务启动完成 ===" | tee -a $LOG/steps.log

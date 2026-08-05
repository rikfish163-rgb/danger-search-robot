cmake_minimum_required(VERSION 3.0.2)
project(quadruped_vision)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  message_filters
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

# 本包仅定义二维检测阶段使用的消息。
# 三维观测和已确认目标消息由下游功能包定义。
add_message_files(
  FILES
    Detection.msg
    DetectionArray.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    message_filters
    rospy
    sensor_msgs
    std_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS
    scripts/yolo_detector.py
    scripts/rgbd_collector.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
    config
    launch
    requirements
    weights
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

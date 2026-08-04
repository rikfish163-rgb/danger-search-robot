cmake_minimum_required(VERSION 3.0.2)
project(danger_target_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  rospy
  std_msgs
  std_srvs
)

catkin_python_setup()

add_message_files(
  FILES
    DangerObservation.msg
    ConfirmedDanger.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    rospy
    std_msgs
    std_srvs
)

catkin_install_python(
  PROGRAMS
    scripts/target_manager_node.py
    scripts/result_writer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  catkin_add_nosetests(test/test_core.py)
  catkin_add_nosetests(test/test_scenarios.py)
  add_rostest(test/target_manager.test)
endif()

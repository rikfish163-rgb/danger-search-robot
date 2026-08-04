cmake_minimum_required(VERSION 3.0.2)
project(vision_localization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(
  -Wall
  -Wextra
)

find_package(catkin REQUIRED COMPONENTS
  quadruped_vision
  danger_target_manager
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED COMPONENTS
  common
  sample_consensus
  segmentation
)

catkin_package(
  INCLUDE_DIRS
    include

  CATKIN_DEPENDS
    quadruped_vision
    danger_target_manager
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    message_filters
    tf2
    tf2_ros
    tf2_geometry_msgs
    pcl_ros
    pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_library(pointcloud_roi_lib
  src/pointcloud_roi.cpp
  src/localization_config.cpp
  src/input_adapter.cpp
  src/coordinate_estimator.cpp
  src/sphere_ransac.cpp
  src/tf_transformer.cpp
  src/output_adapter.cpp
)

target_link_libraries(pointcloud_roi_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(
  pointcloud_roi_lib
  ${catkin_EXPORTED_TARGETS}
)

add_executable(danger_localization_node
  src/danger_localization_node.cpp
)

target_link_libraries(danger_localization_node
  pointcloud_roi_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_dependencies(
  danger_localization_node
  ${catkin_EXPORTED_TARGETS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    test_pointcloud_roi
    test/test_pointcloud_roi.launch
    test/test_pointcloud_roi.cpp
  )

  target_link_libraries(
    test_pointcloud_roi
    pointcloud_roi_lib
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )

  add_dependencies(
    test_pointcloud_roi
    ${catkin_EXPORTED_TARGETS}
  )

  catkin_add_gtest(
    test_sphere_ransac
    test/test_sphere_ransac.cpp
  )

  if(TARGET test_sphere_ransac)
    target_link_libraries(
      test_sphere_ransac
      pointcloud_roi_lib
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
    )

    add_dependencies(
      test_sphere_ransac
      ${catkin_EXPORTED_TARGETS}
    )
  endif()
endif()

install(
  TARGETS
    danger_localization_node
    pointcloud_roi_lib

  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
    rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

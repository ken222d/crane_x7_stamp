# このファイルはKeitaro NakamuraとRyotaro Karikomiによって作成され, その後Akira Matsumotoによって変更されました.
# 変更内容：fixed_position_publisherとopen_close_handの追加
cmake_minimum_required(VERSION 3.5)
project(crane_x7_stamp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(fixed_position_publisher
  src/fixed_position_publisher.cpp
)
ament_target_dependencies(fixed_position_publisher
    rclcpp
    geometry_msgs
    tf2_ros
)
add_executable(press_the_stamp
  src/press_the_stamp.cpp
)
ament_target_dependencies(press_the_stamp
    cv_bridge
    geometry_msgs
    image_geometry
    moveit_ros_planning_interface
    OpenCV
    rclcpp
    tf2_geometry_msgs
)

add_executable(open_close_hand
  src/open_close_hand.cpp
)
ament_target_dependencies(open_close_hand
    cv_bridge
    geometry_msgs
    image_geometry
    moveit_ros_planning_interface
    OpenCV
    rclcpp
    tf2_geometry_msgs
)

add_executable(press_the_stamp_tf
  src/press_the_stamp_tf.cpp
)
ament_target_dependencies(press_the_stamp_tf
    cv_bridge
    geometry_msgs
    image_geometry
    moveit_ros_planning_interface
    OpenCV
    rclcpp
    tf2_geometry_msgs
)

# src下のソースコードはTARGETSで
# launch下のlaunchファイルはDIRECTORYで
install(TARGETS
  fixed_position_publisher
  DESTINATION lib/${PROJECT_NAME}/
)
install(TARGETS
  press_the_stamp
  DESTINATION lib/${PROJECT_NAME}/
) 

install(TARGETS
  open_close_hand
  DESTINATION lib/${PROJECT_NAME}/
)

install(TARGETS
  press_the_stamp_tf
  DESTINATION lib/${PROJECT_NAME}/
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.0.2)
project(arm_kinematics)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  trajectory_msgs
  sensor_msgs
  tf
  tf2
  tf2_ros
  xacro
  robot_state_publisher
  joint_state_publisher
  gazebo_ros
  controller_manager
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    trajectory_msgs
    sensor_msgs
    tf
    tf2
    tf2_ros
    robot_state_publisher
    joint_state_publisher
    gazebo_ros
    controller_manager
)

catkin_install_python(PROGRAMS
  scripts/tf2_broadcaster.py
  scripts/inverse_kinematics.py
  scripts/trajectory_planner.py
  scripts/arm_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

cmake_minimum_required(VERSION 3.0.2)
project(rb_theron_command_manager)

find_package(catkin REQUIRED COMPONENTS
  robot_simple_command_manager
  robot_simple_command_manager_msgs
  rospy
  message_runtime
  std_msgs
  std_srvs
  tf
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install( DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        FILES_MATCHING PATTERN "*.yaml"
)

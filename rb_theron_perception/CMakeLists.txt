cmake_minimum_required(VERSION 2.8.3)
project(rb_theron_perception)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(
  DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

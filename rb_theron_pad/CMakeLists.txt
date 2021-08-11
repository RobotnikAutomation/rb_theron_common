cmake_minimum_required(VERSION 2.8.3)
project(rb_theron_pad)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  geometry_msgs
  joy
  message_generation
  robotnik_msgs
  roscpp
  sensor_msgs
)

add_service_files(DIRECTORY srv
   FILES
    enable_disable_pad.srv
)

generate_messages(
   DEPENDENCIES
   robotnik_msgs sensor_msgs
 
)

catkin_package(
    CATKIN_DEPENDS
    diagnostic_updater
        geometry_msgs
        joy
        message_runtime
        robotnik_msgs
        roscpp
        sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rb_theron_pad src/rb_theron_pad.cpp)

add_dependencies(rb_theron_pad ${catkin_EXPORTED_TARGETS} ${rb_theron_pad_EXPORTED_TARGETS})

target_link_libraries(rb_theron_pad
   ${catkin_LIBRARIES}
 )

install(
  TARGETS rb_theron_pad
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

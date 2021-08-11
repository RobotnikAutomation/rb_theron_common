cmake_minimum_required(VERSION 2.8.3)
project(rb_theron_localization)

find_package(catkin REQUIRED COMPONENTS
    amcl
        geometry_msgs
        gmapping
        imu_complementary_filter
        map_server
        mavros_msgs
        message_filters
        robot_localization
        robotnik_msgs
        roscpp
        rospy
        std_msgs
        std_srvs
        tf
        tf2
        tf2_geometry_msgs
        tf2_ros
)

catkin_package(
    CATKIN_DEPENDS
    geometry_msgs
        mavros_msgs
        message_filters
        robot_localization
        robotnik_msgs
        roscpp
        rospy
        std_msgs
        std_srvs
        tf
        tf2
        tf2_geometry_msgs
        tf2_ros
)

install(
  DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY maps
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


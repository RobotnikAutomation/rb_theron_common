cmake_minimum_required(VERSION 2.8.3)
project(rb_theron_navigation)

find_package(catkin REQUIRED COMPONENTS
	move_base
	spatio_temporal_voxel_layer
	teb_local_planner
        global_planner
)

catkin_package(

)

install(
  DIRECTORY launch config_rviz config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

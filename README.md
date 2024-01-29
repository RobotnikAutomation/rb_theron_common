# rb_theron_common (with RB-THERON ROS 1 Noetic)
The `rb_theron_common` repository contains common packages for the control and simulation of the RB-THERON robot, such as the URDF description of the RB-THERON, platform messages and other files for simulation.



## Packages

### rb_theron_common
The rb_theron_common is a metapackage. It contains RB-THERON common packages used for robot control and for simulation.
The rb_theron_common depends on the following packages: rb_theron_control, rb_theron_description, rb_theron_localization, rb_theron_navigation and rb_theron_pad.

### rb_theron_common_private
The rb_theron_common_private is a metapackage. It contains RB-THERON common packages with private dependencies used for robot control and for simulation, which are: rb_theron_control, rb_theron_description, rb_theron_localization, rb_theron_navigation, rb_theron_navigation_private, rb_theron_pad, rb_theron_perception and rb_theron_robot_local_control.

### rb_theron_control
The rb_theron_control package for real or simulated robot control configuration files. It contains the launch and configuration files to spawn the joint controllers with the ROS controller_manager.

### rb_theron_description
The urdf, meshes, and other elements needed in the description are contained here. This package includes the description of the RB-THERON mobile platforms. The package includes also some launch files to publish the robot state and to test the urdf files in rviz.

### rb_theron_localization
Contains the configuration and launch files to use the robot localization packages along the real or simulated robot.

### rb_theron_navigation
Contains the configuration and launch files to work with the ROS navigation stack along the real or simulated robot.

### rb_theron_navigation_private
This package configures the robotnik_navigation private packages.

### rb_theron_pad
This package contains the node that subscribes to /joy messages and publishes command messages for the robot platform including speed level control. The joystick output is feed to a [mux](http://wiki.ros.org/twist_mux) so that the final command to the robot can be set by different components (move_base, etc.)
The node allows to load different types of joysticks (PS4, PS3, Logitech, Thrustmaster). New models can be easily added by creating new .yaml files.

### rb_theron_perception
The rb_theron_perception package for perception configuration files.

### rb_theron_robot_local_control
The rb_theron_robot_local package for configuration files of the local robot, such as the topics frames, the frequency of the messages, the type of the components, etc.

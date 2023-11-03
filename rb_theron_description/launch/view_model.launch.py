# Copyright (c) 2023, Robotnik Automation S.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Shows the robot in RViz2 and publishes the robot state from joint_state_publisher_gui"""

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Returns the launch description"""
    ret_ld = LaunchDescription()

    ret_ld.add_action(
        GroupAction(
            [
                PushRosNamespace("robot"),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare("rb_theron_description"),
                                "launch",
                                "default.launch.py",
                            ]
                        )
                    ),
                    launch_arguments={
                        "use_sim_time": "false",
                        "robot_id": "robot",
                        "controller_path": "''",
                    }.items(),
                ),
                Node(
                    package="joint_state_publisher_gui",
                    executable="joint_state_publisher_gui",
                    on_exit=Shutdown(),
                ),
                Node(
                    package="rviz2",
                    executable="rviz2",
                    arguments=[
                        "-d",
                        PathJoinSubstitution(
                            [
                                FindPackageShare("rb_theron_description"),
                                "rviz",
                                "default.rviz",
                            ]
                        ),
                    ],
                    on_exit=Shutdown(),
                ),
            ]
        )
    )

    return ret_ld

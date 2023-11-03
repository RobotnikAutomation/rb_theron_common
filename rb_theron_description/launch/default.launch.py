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

"""Proccess xacro file of the robot and publish the robot state"""

from launch import LaunchDescription
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from robotnik_common.launch import AddArgumentParser, ExtendedArgument


def generate_launch_description():
    """Returns the launch description"""
    ret_ld = LaunchDescription()
    add_to_launcher = AddArgumentParser(ret_ld)

    add_to_launcher.add_arg(
        ExtendedArgument(
            name="use_sim_time",
            description="Use simulation/Gazebo clock",
            default_value="false",
            use_env=True,
            environment="USE_SIM_TIME",
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name="tf_prefix",
            description="Prefix for links and joints",
            default_value="",
            use_env=True,
            environment="TF_PREFIX",
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name="model",
            description="Model of the robot",
            default_value="default",
            use_env=True,
            environment="ROBOT_MODEL",
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name="description_path",
            description="Path to the robot description",
            default_value=[
                PathJoinSubstitution([FindPackageShare("rb_theron_description")]),
                "/robots/",
                LaunchConfiguration("model"),
                ".urdf.xacro",
            ],
            use_env=True,
            environment="ROBOT_DESCRIPTION_PATH",
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name="namespace",
            description="Namespace of the nodes",
            default_value="",
            use_env=False,
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name="controller_path",
            description="Path to the file containing the controllers configuration (Gazebo only)",
            default_value='""',
            use_env=True,
            environment="ROBOT_CONTROLLER_PATH",
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name="robot_id",
            description="Robot ID",
            default_value="robot",
            use_env=True,
            environment="ROBOT_ID",
        )
    )

    add_to_launcher.add_arg(
        ExtendedArgument(
            name="namespace",
            description="Namespace of the nodes",
            default_value=LaunchConfiguration("robot_id"),
            use_env=True,
            environment="NAMESPACE",
        )
    )

    params = add_to_launcher.process_arg()

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            params["description_path"],
            " namespace:=",
            params["namespace"],
            " config_controllers:=",
            params["controller_path"],
        ]
    )
    robot_description_content = ParameterValue(
        robot_description_content, value_type=str
    )

    ret_ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "use_sim_time": params["use_sim_time"],
                    "robot_description": robot_description_content,
                    "publish_frequency": 100.0,
                    "frame_prefix": [params["robot_id"], "_"],
                }
            ],
        )
    )

    return ret_ld

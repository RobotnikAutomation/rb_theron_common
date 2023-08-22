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
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from ament_index_python.packages import get_package_share_directory

from os.path import join

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from robotnik_common.launch import RewrittenYaml
from robotnik_common.launch import ExtendedArgument
from robotnik_common.launch import AddArgumentParser


def generate_launch_description():
    ld = LaunchDescription()
    add_to_launcher = AddArgumentParser(ld)

    arg = ExtendedArgument(
        name='use_sim_time',
        description='Use simulation/Gazebo clock',
        default_value='true',
        use_env=True,
        environment='use_sim_time',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='robot_id',
        description='Robot ID',
        default_value='robot',
        use_env=True,
        environment='ROBOT_ID',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='namespace',
        description='Namespace of the nodes',
        default_value=LaunchConfiguration('robot_id'),
        use_env=True,
        environment='NAMESPACE',
    )
    add_to_launcher.add_arg(arg)

    def_slam_param_desc = "Full path to the ROS2 parameters file to use "
    def_slam_param_desc += "for the slam_toolbox node"
    def_slam_param_file = join(
        get_package_share_directory("rb_theron_localization"),
        "config",
        "slam_params.yaml"
    )
    arg = ExtendedArgument(
        name='slam_params_file',
        description=def_slam_param_desc,
        default_value=def_slam_param_file,
        use_env=True,
        environment='SLAM_PARAMS_FILE',
    )
    add_to_launcher.add_arg(arg)

    params = add_to_launcher.process_arg()

    config_file = RewrittenYaml(
        source_file=params['slam_params_file'],
        param_rewrites={
            'use_sim_time': params['use_sim_time'],
            'odom_frame': [params['robot_id'], '/odom'],
            'base_frame': [params['robot_id'], '/base_footprint'],
            'map_frame': 'map',
            'scan_topic': [params['robot_id'], '/laser/scan']
        },
        # root_key=[
        #     params['namespace']
        # ],
        # root_key=[''],
        convert_types=True,
    )

    ld.add_action(
        PushRosNamespace(
            namespace=params['namespace']
        )
    )

    ld.add_action(
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[config_file],
        )
    )

    return ld

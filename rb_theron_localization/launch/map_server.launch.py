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

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

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

    arg = ExtendedArgument(
        name='map_name',
        description='Name of the map file',
        default_value='opil_factory',
        use_env=True,
        environment='MAP_NAME',
    )
    add_to_launcher.add_arg(arg)

    def_map_file_abs = [
        get_package_share_directory('rb_theron_localization'),
        '/maps/',
        LaunchConfiguration('map_name'),
        '/map.yaml'
    ]
    arg = ExtendedArgument(
        name='map_file_abs',
        description='Absolute path to the map file',
        default_value=def_map_file_abs,
        use_env=True,
        environment='MAP_FILE_ABS',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='map_frame_id',
        description='Frame id of the map',
        default_value=[
            LaunchConfiguration('robot_id'),
            '/',
            'map',
        ],
        use_env=True,
        environment='MAP_FRAME_ID',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='topic_name',
        description='topic of the map',
        default_value='map',
        use_env=True,
        environment='TOPIC_NAME',
    )
    add_to_launcher.add_arg(arg)

    params = add_to_launcher.process_arg()

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        # namespace=params['namespace'],
        parameters=[
            {
                'use_sim_time': params['use_sim_time'],
                'yaml_filename': params['map_file_abs'],
                'topic_name': params['topic_name'],
                'frame_id': params['map_frame_id'],
            }
        ],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        # namespace=params['namespace'],
        output='screen',
        parameters=[
            {
                'use_sim_time': params['use_sim_time'],
                'autostart': True,
                'node_names': ['map_server'],
            }
        ],
    )

    ld.add_action(
        PushRosNamespace(
            namespace=params['namespace']
        )
    )
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager)

    return ld

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

    default_map_name = 'opil_factory'
    arg = ExtendedArgument(
        name='map_name',
        description='Name of the map file',
        default_value=default_map_name,
        use_env=True,
        environment='MAP_NAME',
    )
    add_to_launcher.add_arg(arg)

    def_amcl_file = [
        get_package_share_directory('rb_theron_localization'),
        '/config/amcl.yaml'
    ]

    arg = ExtendedArgument(
        name='amcl_file',
        description='Absolute path to the amcl file',
        default_value=def_amcl_file,
        use_env=True,
        environment='AMCL_FILE',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='map_frame_id',
        description='Frame id of the map',
        default_value='map',
        use_env=True,
        environment='MAP_FRAME_ID',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='scan_topic',
        description='2D laser scan topic to use',
        default_value='mergered_laser/scan',
        use_env=True,
        environment='SCAN_TOPIC',
    )
    add_to_launcher.add_arg(arg)
    params = add_to_launcher.process_arg()

    lifecycle_nodes = ['amcl']

    amcl_rewritten = RewrittenYaml(
        source_file=params['amcl_file'],
        param_rewrites={
            'use_sim_time': params['use_sim_time'],
            'base_frame_id': [params['robot_id'], '/base_footprint'],
            'global_frame_id': [
                params['robot_id'],
                '/',
                params['map_frame_id']
            ],
            'odom_frame_id': [params['robot_id'], '/odom'],
            'scan_topic': params['scan_topic'],
        },
        root_key=[params['namespace']],
        convert_types=True,
    )
    print(amcl_rewritten)

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[
            {
                'use_sim_time': params['use_sim_time']
            },
            amcl_rewritten
        ],
        remappings=[
            (
                'map',
                [
                    params['map_frame_id']
                ],
            ),
        ],
        output='screen',
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': params['use_sim_time'],
                'autostart': True,
                'node_names': lifecycle_nodes
            }
        ]
    )

    ld.add_action(
        PushRosNamespace(
            namespace=params['namespace']
        )
    )
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager)

    return ld

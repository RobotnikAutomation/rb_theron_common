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
from launch.actions import SetEnvironmentVariable
from launch.actions import GroupAction
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

    arg = ExtendedArgument(
        name='map_frame_id',
        description='Frame id of the map',
        default_value='map',
        use_env=True,
        environment='MAP_FRAME_ID',
    )
    add_to_launcher.add_arg(arg)

    def_nav_file = [
        get_package_share_directory(
            'rb_theron_navigation'
        ),
        '/config/nav.yaml',
    ]

    arg = ExtendedArgument(
        name='nav_config_file',
        description='Absolute path to the nav config file',
        default_value=def_nav_file,
        use_env=True,
        environment='NAV_CONFIG_FILE',
    )
    add_to_launcher.add_arg(arg)

    params = add_to_launcher.process_arg()

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    remappings = [
        (
            '/front_laser/scan',
            [
                '/',
                params['namespace'],
                '/front_laser/scan',
            ]
        ),
        (
            '/rear_laser/scan',
            [
                '/', params['namespace'],
                '/rear_laser/scan',
            ]
        ),
    ]

    configured_params = RewrittenYaml(
        source_file=params['nav_config_file'],
        root_key=params['namespace'],
        param_rewrites={
            'use_sim_time': params['use_sim_time'],
            'robot_base_frame': [
                params['robot_id'],
                '/base_footprint'
            ],
            'odom_topic': [
                params['robot_id'],
                '/robotnik_base_control/odom',
            ],
            'global_frame': [
                params['robot_id'],
                '/',
                params['map_frame_id'],
            ],

        },
        convert_types=True
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM',
        '1'
    )

    load_nodes = GroupAction(
        actions=[
            PushRosNamespace(
                namespace=params['namespace']
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    configured_params
                ],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    configured_params
                ],
                remappings=remappings
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    configured_params,
                ],
                remappings=remappings
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                     configured_params
                ],
                remappings=remappings
                + [
                    (
                        'cmd_vel',
                        'cmd_vel_nav',
                    )
                ],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    {
                        'use_sim_time': params['use_sim_time']
                    },
                    configured_params
                ],
                remappings=remappings
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    configured_params
                ],
                remappings=remappings
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn_delay=2.0,
                parameters=[
                    configured_params
                ],
                remappings=remappings
                + [
                    (
                        'cmd_vel',
                        'cmd_vel_nav',
                    ),
                    (
                        'cmd_vel_smoothed',
                        [
                            '/',
                            params['namespace'],
                            '/robotnik_base_control/cmd_vel_unstamped',
                        ]
                    ),
                ]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                    {
                        'use_sim_time': params['use_sim_time'],
                        'autostart': True,
                        'node_names': lifecycle_nodes
                    }
                ]
            ),
        ]
    )

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(load_nodes)

    return ld

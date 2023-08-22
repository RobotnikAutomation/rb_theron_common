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
        name='base_frame',
        description='Base frame',
        default_value='base_link',
        use_env=True,
        environment='BASE_FRAME',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='laserscan_topics',
        description='Laser scan topics',
        default_value='front_laser/scan rear_laser/scan',
        use_env=True,
        environment='LASERSCAN_TOPICS',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='topic_output_prefix',
        description='Topic output prefix',
        default_value='mergered_laser',
        use_env=True,
        environment='TOPIC_OUTPUT_PREFIX',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='angle_min',
        description='Minimum angle',
        default_value='-3.141592653589793',
        use_env=True,
        environment='ANGLE_MIN',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='angle_max',
        description='Maximum angle',
        default_value='3.141592653589793',
        use_env=True,
        environment='ANGLE_MAX',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='angle_increment',
        description='Angle increment',
        default_value='0.005',
        use_env=True,
        environment='ANGLE_INCREMENT',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='scan_time',
        description='Scan time',
        default_value='0.0',
        use_env=True,
        environment='SCAN_TIME',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='range_min',
        description='Minimum range',
        default_value='0.1',
        use_env=True,
        environment='RANGE_MIN',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='range_max',
        description='Maximum range',
        default_value='20.0',
        use_env=True,
        environment='RANGE_MAX',
    )
    add_to_launcher.add_arg(arg)

    params = add_to_launcher.process_arg()

    ld.add_action(
        PushRosNamespace(
            namespace=params['namespace']
        )
    )
    ld.add_action(
        Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_merger',
            output='screen',
            parameters=[
                {
                    'use_sim_time': params['use_sim_time'],
                    'destination_frame': [
                        params['robot_id'],
                        '/base_link'
                    ],
                    'cloud_destination_topic': [
                        params['topic_output_prefix'],
                        '/points'
                    ],
                    'scan_destination_topic': [
                        params['topic_output_prefix'],
                        '/scan'
                    ],
                    'laserscan_topics': params['laserscan_topics'],
                    'angle_min': params['angle_min'],
                    'angle_max': params['angle_max'],
                    'angle_increment': params['angle_increment'],
                    'scan_time': params['scan_time'],
                    'range_min': params['range_min'],
                    'range_max': params['range_max'],
                }
            ],
        )
    )

    return ld

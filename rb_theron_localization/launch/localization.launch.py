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

import launch, launch_ros, os
from ament_index_python.packages import get_package_share_directory
from robotnik_common.launch import RewrittenYaml, add_launch_args

def generate_launch_description():

  ld = launch.LaunchDescription()
  p = [
    ('use_sim_time', 'Use simulation/Gazebo clock', 'true'),
    ('robot_id', 'Frame id of the sensor', 'robot'),
    ('namespace', 'Namespace of the nodes', launch.substitutions.LaunchConfiguration('robot_id')),
    ('amcl_file', 'Absolute path to the amcl file', [get_package_share_directory('rb_theron_localization'), '/config/amcl.yaml']),
    ('map_frame_id', 'Frame id of the map', 'map'),
  ]
  params = add_launch_args(ld, p)

  amcl_rewritten = RewrittenYaml(
      source_file=params['amcl_file'],
      param_rewrites={
          'use_sim_time': params['use_sim_time'],
          'base_frame_id': [params['robot_id'], '/base_footprint'],
          'global_frame_id': params['map_frame_id'],
          'odom_frame_id': [params['robot_id'], '/odom'],
          'scan_topic': ['laser/scan'],
      },
      root_key=[params['namespace']],
      convert_types=True,
  )

  ld.add_action(launch_ros.actions.Node(
    namespace=params['namespace'],
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    parameters=[amcl_rewritten],
    remappings=[
      ('map', '/map'),
    ],
    output='screen'
  ))

  ld.add_action(launch_ros.actions.Node(
    namespace=params['namespace'],
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_localization',
    output='screen',
    parameters=[{
      'use_sim_time': params['use_sim_time'],
      'autostart': True,
      'node_names': ['amcl'],
    }],
  ))

  return ld
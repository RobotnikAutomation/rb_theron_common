# Copyright (c) 2023, Robotnik Automation S.L.L.
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

def read_params(ld : launch.LaunchDescription, params : list[tuple[str, str, str]]): # name, description, default_value

  # Declare the launch options
  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='environment',
    description='Read parameters from environment variables',
    choices=['true', 'false'],
    default_value='true',
  ))
  for param in params:
    ld.add_action(launch.actions.DeclareLaunchArgument(
      name=param[0], description=param[1], default_value=param[2],))

  # Get the launch configuration variables
  ret={}
  if launch.substitutions.LaunchConfiguration('environment') == 'false':
    for param in params:
      ret[param[0]] = launch.substitutions.LaunchConfiguration(param[0])
  else:
    for param in params:
      if str.upper(param[0]) in os.environ:
        ret[param[0]] = launch.substitutions.EnvironmentVariable(str.upper(param[0]))
      else: ret[param[0]] = launch.substitutions.LaunchConfiguration(param[0])

  return ret


def generate_launch_description():
  ld = launch.LaunchDescription()
  p = [
    ('use_sim_time', 'Use simulation/Gazebo clock', 'true'),
    ('robot_id', 'Name of the robot', 'robot'),
    ('slam_params_file', 'Full path to the ROS2 parameters file to use for the slam_toolbox node', os.path.join(get_package_share_directory("rb_theron_localization"), "config", "slam_params.yaml")),
  ]
  params = add_launch_args(ld, p)

  config_file = RewrittenYaml(
    source_file=params['slam_params_file'],
    param_rewrites={
      'use_sim_time': params['use_sim_time'],
      'odom_frame': [params['robot_id'], '/odom'],
      'base_frame': [params['robot_id'], '/base_footprint'],
      'map_frame': 'map',
      'scan_topic': [params['robot_id'], '/laser/scan']
    },
#    root_key=[params['namespace']],
#    root_key=[''],
    convert_types=True,
  )

# ld.add_action(launch_ros.actions.PushRosNamespace(namespace='robot'))
  ld.add_action(launch_ros.actions.Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    parameters=[config_file],
  ))

  return ld

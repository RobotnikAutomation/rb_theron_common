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

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

from robotnik_common.launch import RewrittenYaml

#from launch import LaunchDescription
#from launch.actions import DeclareLaunchArgument
#from launch.substitutions import LaunchConfiguration
#from launch_ros.actions import Node
#from ament_index_python.packages import get_package_share_directory


def read_params(ld : launch.LaunchDescription):
  environment = launch.substitutions.LaunchConfiguration('environment')
  use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
  robot_id = launch.substitutions.LaunchConfiguration('robot_id')
  slam_params_file = launch.substitutions.LaunchConfiguration('slam_params_file')

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='environment',
    description='Read parameters from environment variables',
    choices=['true', 'false'],
    default_value='true',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='use_sim_time',
    description='Use simulation/Gazebo clock',
    choices=['true', 'false'],
    default_value='true',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='robot_id',
    description='Name of the robot',
    default_value='robot',
  ))

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='slam_params_file',
    description='Full path to the ROS2 parameters file to use for the slam_toolbox node',
    default_value=os.path.join(get_package_share_directory("rb_theron_localization"), "config", "slam_params.yaml"),
  ))

  ret = {}

  if environment == 'false':
    ret = {
      'use_sim_time': use_sim_time,
      'slam_params_file': slam_params_file,
    }
  else:
    if 'USE_SIM_TIME' in os.environ:
      ret['use_sim_time'] = os.environ['USE_SIM_TIME']
    else: ret['use_sim_time'] = use_sim_time

    if 'ROBOT_ID' in os.environ:
      ret['robot_id'] = os.environ['ROBOT_ID']
    else: ret['robot_id'] = robot_id

    if 'SLAM_PARAMS_FILE' in os.environ:
      ret['slam_params_file'] = os.environ['SLAM_PARAMS_FILE']
    else: ret['slam_params_file'] = slam_params_file

  return ret


def generate_launch_description():
  ld = launch.LaunchDescription()
  params = read_params(ld)

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

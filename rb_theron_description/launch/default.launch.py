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

import os, launch, launch_ros

def read_params(ld : launch.LaunchDescription, params : list[tuple[str, str, str]]): # name, description, default_value

  # Declare the launch options
  for param in params:
    ld.add_action(launch.actions.DeclareLaunchArgument(
      name=param[0], description=param[1], default_value=param[2],))

  # Get the launch configuration variables
  ret={}
  for param in params:
    ret[param[0]] = launch.substitutions.LaunchConfiguration(param[0])

  return ret


def generate_launch_description():

  ld = launch.LaunchDescription()
  p = [
    ('use_sim_time', 'Use simulation (Gazebo) clock if true', 'true'),
    ('robot_description_file', 'Name of the file containing the robot description', 'rb_theron.urdf.xacro'),
    ('robot_description_path', 'Path to the file containing the robot description', [launch_ros.substitutions.FindPackageShare('rb_theron_description'), '/robots/', launch.substitutions.LaunchConfiguration('robot_description_file')]),
    ('robot_id', 'Id of the robot', 'robot'),
    ('controller_path', 'Path to the file containing the controllers configuration', '\" \"'),
  ]
  params = read_params(ld, p)

  robot_description_content = launch.substitutions.Command([
    launch.substitutions.PathJoinSubstitution([launch.substitutions.FindExecutable(name="xacro")]),
    ' ', params['robot_description_path'],
    ' robot_id:=', params['robot_id'],
    ' robot_ns:=', params['robot_id'],
    ' config_controllers:=', params['controller_path'],
  ])
  # Create parameter
  robot_description_param = launch_ros.descriptions.ParameterValue(robot_description_content, value_type=str)

  ld.add_action(launch_ros.actions.Node(
    namespace=params['robot_id'],
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
      'use_sim_time': params['use_sim_time'],
      'robot_description': robot_description_param,
      'publish_frequency': 100.0,
      'frame_prefix': [params['robot_id'], '/'],
    }],
  ))

  return ld

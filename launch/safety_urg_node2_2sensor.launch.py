# Copyright 2024 HOKUYO AUTOMATIC CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import launch
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

pkg_name = 'safety_urg_node2'

def generate_launch_description():

    # パラメータファイルのパス設定
    config_file_path_1 = os.path.join(
        get_package_share_directory('hokuyo_navigation2'),
        'config',
        'uam1_param.yaml'
    )
    config_file_path_2 = os.path.join(
        get_package_share_directory('hokuyo_navigation2'),
        'config',
        'uam2_param.yaml'
    )

    return LaunchDescription([
        Node(
            package = pkg_name,
            executable = 'safety_urg_node2',
            name = 'safety_urg_node2_1',
            parameters = [config_file_path_1],
        ),
        Node(
            package = pkg_name,
            executable = 'safety_urg_node2',
            name = 'safety_urg_node2_2',
            parameters = [config_file_path_2],
        ),
    ])

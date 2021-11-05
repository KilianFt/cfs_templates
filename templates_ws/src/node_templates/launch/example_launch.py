# Copyright 2021 CFS
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
from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription()

    config = os.path.join(
        get_package_share_directory('node_templates'),
        'config',
        'params.yaml'
    )

    example_node = launch_ros.actions.Node(
        package='node_templates',
        executable='subpub_example',
        output='screen',
        name='subpub_example',
        parameters=[config]
    )

    ld.add_action(example_node)

    return ld

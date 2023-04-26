# Copyright 2019 Intelligent Robotics Lab
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_gpsr_tayros2')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/house_granny_domain.pddl',
          'namespace': namespace
          }.items())

    pick_request_attend_cmd = Node(
       package='plansys2_gpsr_tayros2',
       executable='pick_request_attend_node',
       name='attend_pick_request',
       namespace=namespace,
       output='screen',
       parameters=[])

    open_door_cmd = Node(
       package='plansys2_gpsr_tayros2',
       executable='open_door_action_node',
       name='open_door_action_node',
       namespace=namespace,
       output='screen',
       parameters=[])

    move_without_door_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_wod',
        namespace=namespace,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'move_without_door',
                'publisher_port': 1676,
                'server_port': 1677,
                'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
            }
        ])

    move_with_door_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_wd',
        namespace=namespace,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'move_by_door',
                'publisher_port': 1678,
                'server_port': 1679,
                'bt_xml_file': example_dir + '/behavior_trees_xml/move_wd.xml'
            }
        ])

    pick_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='pick',
        namespace=namespace,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'pick',
                'publisher_port': 1682,
                'server_port': 1683,
                'bt_xml_file': example_dir + '/behavior_trees_xml/pick.xml'
            }
        ])

    drop_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='drop',
        namespace=namespace,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'drop',
                'publisher_port': 1684,
                'server_port': 1685,
                'bt_xml_file': example_dir + '/behavior_trees_xml/drop.xml'
            }
        ])

    # -- Only if no doors map
    # transport_cmd = Node(
    #     package='plansys2_bt_actions',
    #     executable='bt_action_node',
    #     name='transport',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[
    #         example_dir + '/config/params.yaml',
    #         {
    #         'action_name': 'transport',
    #         'publisher_port': 1682,
    #         'server_port': 1683,
    #         'bt_xml_file': example_dir + '/behavior_trees_xml/transport.xml'
    #         }
    #     ])

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(open_door_cmd)
    ld.add_action(move_without_door_cmd)
    ld.add_action(move_with_door_cmd)
    ld.add_action(pick_cmd)
    ld.add_action(drop_cmd)
    ld.add_action(pick_request_attend_cmd)
    # ld.add_action(transport_cmd)

    return ld

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

    # Specify the actions
    close_door_cmd = Node(
        package='plansys2_gpsr_tayros2',
        executable='close_door_action_node',
        name='close_door_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])

    #open_door_cmd = Node(
    #    package='plansys2_gpsr_tayros2',
    #    executable='open_door_action_node',
    #    name='open_door_action_node',
    #    namespace=namespace,
    #    output='screen',
    #    parameters=[])

    # move_by_door_cmd = Node(
    #     package='plansys2_gpsr_tayros2',
    #     executable='move_by_door_action_node',
    #     name='move_by_door_action_node',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[])
    
    # pick_cmd = Node(
    #     package='plansys2_gpsr_tayros2',
    #     executable='pick_action_node',
    #     name='pick_action_node',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[])
    
    #pick_prio_cmd = Node(
    #    package='plansys2_gpsr_tayros2',
    #    executable='pick_prio_action_node',
    #    name='pick_prio_action_node',
    #    namespace=namespace,
    #    output='screen',
    #    parameters=[])
    
    #drop_cmd = Node(
    #    package='plansys2_gpsr_tayros2',
    #    executable='drop_action_node',
    #    name='drop_action_node',
    #    namespace=namespace,
    #    output='screen',
    #    parameters=[])

    # Both are the same action but 1 has to be done first
    transport_cmd = Node(
        package='plansys2_gpsr_tayros2',
        executable='bt_action_node',
        name='transport',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'transport',
            'publisher_port': 1674,
            'server_port': 1675,
            'bt_xml_file': example_dir + '/behavior_trees_xml/transport.xml'
          }
        ])

    move_without_door_cmd = Node(
        package='plansys2_gpsr_tayros2',
        executable='bt_action_node',
        name='move_wod',
        namespace=namespace,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
            'action_name': 'move_by_door',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
            }
        ])
    
    move_with_door_cmd = Node(
        package='plansys2_gpsr_tayros2',
        executable='bt_action_node',
        name='move_wd',
        namespace=namespace,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
            'action_name': 'move_without_door',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/move_wd.xml'
            }
        ])
    
    transport_prio_cmd = Node(
        package='plansys2_gpsr_tayros2',
        executable='bt_action_node',
        name='transport_prio',
        namespace=namespace,
        parameters=[
            example_dir + '/config/params.yaml',
            {
            'action_name': 'transport',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/transport.xml'
            }
        ])   
     
    transport_cmd = Node(
        package='plansys2_gpsr_tayros2',
        executable='bt_action_node',
        name='transport',
        namespace=namespace,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
            'action_name': 'transport',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/transport.xml'
            }
        ])
    
    open_door_req_cmd = Node(
        package='plansys2_gpsr_tayros2',
        executable='bt_action_node',
        name='open_door_req',
        namespace=namespace,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
            'action_name': 'open_door_req',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/open_door_req.xml'
            }
        ])
    
    close_door_req_cmd = Node(
        package='plansys2_gpsr_tayros2',
        executable='bt_action_node',
        name='close_door_req',
        namespace=namespace,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
            'action_name': 'close_door_req',
            'publisher_port': 1668,
            'server_port': 1669,
            'bt_xml_file': example_dir + '/behavior_trees_xml/close_door_req.xml'
            }
        ])
   
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(close_door_cmd)
    # ld.add_action(open_door_cmd)
    # ld.add_action(move_by_door_cmd)
    # ld.add_action(pick_cmd)
    # ld.add_action(pick_prio_cmd)
    # ld.add_action(drop_cmd)
    ld.add_action(move_without_door_cmd)
    ld.add_action(move_with_door_cmd)
    ld.add_action(transport_cmd)

    ld.add_action(transport_cmd)
    ld.add_action(transport_prio_cmd)

    ld.add_action(close_door_req_cmd)
    ld.add_action(open_door_req_cmd)

    return ld

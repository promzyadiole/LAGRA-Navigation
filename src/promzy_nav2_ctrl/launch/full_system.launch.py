#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    promzy_llm_dir = get_package_share_directory('promzy_llm')
    promzy_nav_dir = get_package_share_directory('promzy_nav2_ctrl')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([

        # ---- Perception Pipeline ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(promzy_llm_dir, 'launch', 'perception_pipeline.launch.py')
            )
        ),

        # ---- LLM + NL Command Bridge ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(promzy_nav_dir, 'launch', 'llm_navigation.launch.py')
            )
        ),

        # ---- Turtlebot3 Navigation (Nav2 Stack) ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'slam': 'false',
                'map': os.path.join(nav2_bringup_dir, 'maps', 'map.yaml')
            }.items()
        ),

        # ---- Gazebo Simulation (Optional) ----
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('turtlebot3_gazebo'),
        #             'launch', 'turtlebot3_world.launch.py'
        #         )
        #     )
        # ),

    ])

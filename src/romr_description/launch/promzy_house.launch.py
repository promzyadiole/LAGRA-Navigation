# promzy_house.launch.py  (ROS 2 Humble, Gazebo Classic)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('romr_description')

    # --- paths inside your package ---
    urdf_path  = os.path.join(pkg_share, 'urdf',   'romr_robot.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'housepromzy.world')  # rename if your file differs

    # Read URDF so robot_state_publisher can publish TF
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # 1) Bring up Gazebo Classic with your world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        ),
        launch_arguments={'world': world_path}.items()
    )

    # 2) Publish TF from URDF (optional but recommended)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 3) Spawn your robot into Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_romr',
        output='screen',
        arguments=[
            '-entity', 'romr',
            '-file',   urdf_path,
            '-x', '0.0', '-y', '0.0', '-z', '0.2',
            '-Y', '0.0'
        ]
    )

    return LaunchDescription([gazebo, rsp, spawn])


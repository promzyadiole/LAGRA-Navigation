from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
    # Set environment variables for Azure AI services
    azure_endpoint = os.environ.get('AZURE_AI_ENDPOINT', '')
    azure_key = os.environ.get('AZURE_AI_KEY', '')
    azure_model = os.environ.get('AZURE_AI_MODEL', '')

    if not azure_endpoint or not azure_key or not azure_model:
        raise ValueError("Missing Azure environment variables: AZURE_AI_ENDPOINT, AZURE_AI_KEY, AZURE_AI_MODEL")

    return LaunchDescription([
        # Set environment variables
        SetEnvironmentVariable('AZURE_AI_ENDPOINT', azure_endpoint),
        SetEnvironmentVariable('AZURE_AI_KEY', azure_key),
        SetEnvironmentVariable('AZURE_AI_MODEL', azure_model),

        # Launch the llm_command node
        Node(
            package='promzy_nav2_ctrl',  # Name of your package
            executable='llm_command',  # Name of the executable node
            name='llm_nav_bridge',  # Node name in ROS graph
            output='screen',  # Output logs to the terminal
            parameters=[{'use_sim_time': False}],  # or True, depending on simulation
        ),
    ])

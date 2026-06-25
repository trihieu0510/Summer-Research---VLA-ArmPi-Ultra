"""
chat.launch.py — start the ArmPi Ultra agent (the brain) with DeepSeek defaults.

This brings up `arm_agent` so you don't retype the model/base_url every time.
The arm hardware (servo controller) and the chat console are run separately:

    # Terminal 1 — free the serial port, start the arm hardware:
    ~/.stop_ros.sh
    ros2 launch sdk armpi_ultra.launch.py

    # Terminal 2 — the agent (this launch file). Export your key first:
    export LLM_API_KEY="<your-deepseek-key>"
    ros2 launch armpi_voice chat.launch.py

    # Terminal 3 — the chat box you actually type in:
    ros2 run armpi_voice arm_console

Override the model/endpoint (e.g. to use a local Ollama instead) with:
    ros2 launch armpi_voice chat.launch.py base_url:=http://localhost:11434/v1 model:=qwen2.5:3b
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='deepseek-chat'),
        DeclareLaunchArgument('base_url', default_value='https://api.deepseek.com'),
        Node(
            package='armpi_voice',
            executable='arm_agent',
            name='arm_agent',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'model': LaunchConfiguration('model'),
                'base_url': LaunchConfiguration('base_url'),
            }],
        ),
    ])

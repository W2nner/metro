"""ROS 2 Humble: ros2 launch /app/launch/metro.launch.py topic:=/lidar_points."""
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('topic',default_value='/lidar_points'),
        DeclareLaunchArgument('config',default_value=str(root/'config/default.json')),
        ExecuteProcess(cmd=[sys.executable,'-m','metro_detector','ros','--topic',LaunchConfiguration('topic'),
                            '--config',LaunchConfiguration('config')],cwd=str(root),output='screen',
                       additional_env={'OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1'})])

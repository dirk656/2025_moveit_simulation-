from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('panda_moveit_config')
    gazebo_launch = os.path.join(pkg_path, 'launch', 'gazebo.launch.py')
    move_group_launch = os.path.join(pkg_path, 'launch', 'move_group.launch.py')
    rviz_launch = os.path.join(pkg_path, 'launch', 'moveit_rviz.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch)),
        TimerAction(
            period=10.0,
            actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(move_group_launch))]
        ),
        TimerAction(
            period=12.0,
            actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(rviz_launch))]
        ),
    ])

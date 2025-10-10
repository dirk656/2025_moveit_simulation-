from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    moveit_dir = get_package_share_directory('panda_configure')
    rh_planner_dir = get_package_share_directory('moveit_rh_planner')
    
    # 设置环境变量，确保RH规划器配置被找到
    rh_planner_config_path = os.path.join(rh_planner_dir, 'config')
    
    return LaunchDescription([
        # 设置环境变量
        SetEnvironmentVariable(
            name='MOVEIT_PLANNER_CONFIG_PATH',
            value=rh_planner_config_path
        ),
        
        # 启动MoveIt演示
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_dir, 'launch', 'demo.launch.py')
            ),
            launch_arguments={
                'use_rviz': 'true',
                'pipeline': 'rh',  # 使用RH作为后端
                'planning_plugin': 'rh_planner_interface/RHPlannerManager',
                'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/ResolveConstraintFrames default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            }.items()
        ),
    ])

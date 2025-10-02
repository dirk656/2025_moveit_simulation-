import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    moveit_simulation_dir = get_package_share_directory('moveit_simulation')
    panda_configure_dir = get_package_share_directory('panda_configure')
    
    # 设置机器人描述参数
    robot_description_path = os.path.join(
        panda_configure_dir, 'config', 'panda.urdf.xacro'
    )
    
    return LaunchDescription([
        # 设置机器人描述参数
        SetEnvironmentVariable(
            name='ROS_DOMAIN_ID',
            value='0'
        ),
        
        # 启动机器人状态发布器
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': f"{{xacro {robot_description_path}}}"
            }]
        ),
        
        # 启动MoveIt仿真节点
        Node(
            package='moveit_simulation',
            executable='moveit_test',
            name='moveit_test',
            output='screen',
            parameters=[{
                'robot_description': f"{{xacro {robot_description_path}}}"
            }]
        ),
    ])

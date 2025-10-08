from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明启动参数
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to launch RViz",
    )
    
    use_plotter_arg = DeclareLaunchArgument(
        "use_plotter",
        default_value="true",
        description="Whether to launch velocity plotter",
    )
    
    # 使用MoveIt配置工具构建配置
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_configure").to_moveit_configs()
    
    # 生成基本的MoveIt演示启动描述
    ld = generate_demo_launch(moveit_config)
    
    # 添加对比节点
    comparison_node = Node(
        package='moveit_simulation',
        executable='comparison_node',
        name='comparison_node',
        output='screen',
        parameters=[
            {"use_sim_time": True}
        ]
    )
    
    # 添加速度绘图节点（可选）
    plotter_node = Node(
        package='moveit_simulation',
        executable='velocity_plotter.py',
        name='velocity_plotter',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_plotter'))
    )
    
    # 添加服务调用节点（用于触发对比）
    trigger_node = Node(
        package='moveit_simulation',
        executable='trigger_comparison.py',
        name='trigger_comparison',
        output='screen'
    )
    
    # 将节点添加到启动描述中
    ld.add_action(use_rviz_arg)
    ld.add_action(use_plotter_arg)
    ld.add_action(comparison_node)
    ld.add_action(plotter_node)
    ld.add_action(trigger_node)
    
    return ld

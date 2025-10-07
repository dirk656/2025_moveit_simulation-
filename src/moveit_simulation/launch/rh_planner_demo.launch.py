from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to launch RViz",
    )
    
    # 使用MoveIt配置工具构建配置
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_configure").to_moveit_configs()
    
    # 生成基本的MoveIt演示启动描述
    ld = generate_demo_launch(moveit_config)
    
    # 添加滚动时域规划器集成节点
    rh_integration_node = Node(
        package='moveit_simulation',
        executable='rh_moveit_integration',
        name='rh_moveit_integration',
        output='screen',
        parameters=[
            {"use_sim_time": True}
        ]
    )
    
    # 添加简单的测试节点（可选）
    test_node = Node(
        package='moveit_simulation',
        executable='moveit_test',
        name='moveit_test',
        output='screen'
    )
    
    # 将节点添加到启动描述中
    ld.add_action(use_rviz_arg)
    ld.add_action(rh_integration_node)
    # ld.add_action(test_node)  # 可选：添加测试节点
    
    return ld

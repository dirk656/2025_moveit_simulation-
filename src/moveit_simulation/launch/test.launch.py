from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 使用MoveIt配置工具构建配置
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_configure").to_moveit_configs()
    
    # 生成基本的MoveIt演示启动描述
    ld = generate_demo_launch(moveit_config)
    
    # 添加我们的测试节点
    test_node = Node(
        package='moveit_simulation',
        executable='moveit_test',
        name='moveit_test',
        output='screen'
    )
    
    # 将测试节点添加到启动描述中
    ld.add_action(test_node)
    
    return ld

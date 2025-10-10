from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 使用MoveIt配置工具构建配置
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_configure").to_moveit_configs()
    
    # 加载RH规划器配置
    rh_planning_yaml = os.path.join(
        get_package_share_directory('panda_configure'),
        'config',
        'rh_planning.yaml'
    )
    
    # 更新moveit_config以使用RH规划器
    moveit_config.robot_description_planning.update({
        'planning_plugins': ['moveit_rh_planner_plugin/RecedingHorizonPlannerManager'],
        'moveit_rh_planner_plugin': {
            'planning_algorithm': 'RH',
            'max_velocity_scaling_factor': 0.1,
            'max_acceleration_scaling_factor': 0.1,
            'planning_time': 5.0,
            'planning_attempts': 3,
            'max_goal_samples': 10,
            'max_state_sampling_attempts': 4,
            'goal_bias': 0.05,
            'simplify_solutions': True,
            'interpolate': True,
            'smooth': True
        }
    })
    
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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_panda = get_package_share_directory('panda_moveit_config')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # 世界文件路径
    world_file = os.path.join(pkg_panda, 'worlds', 'empty.world')

    # URDF 路径
    xacro_file = os.path.join(pkg_panda, 'config', 'panda.urdf.xacro')

    # 生成 robot_description 参数
    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ', xacro_file
    ])

    # ros2_control 参数
    controller_yaml = os.path.join(pkg_panda, 'config', 'ros2_controllers.yaml')

    # 1️⃣ 启动 Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2️⃣ 发布机器人状态
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 3️⃣ 在 Gazebo 中生成 Panda 模型
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'panda'],
        output='screen'
    )

    # 4️⃣ ros2_control 控制器节点
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description},
                    controller_yaml],
        output='screen'
    )

    # 5️⃣ 控制器加载顺序
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    panda_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    panda_hand_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['panda_hand_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 6️⃣ ROS2-Gazebo 桥接
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/panda/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/world/empty/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/world/empty/control@ros_gz_interfaces/srv/ControlWorld'
        ],
        output='screen'
    )

    # 7️⃣ 启动 MoveIt2 RViz
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_panda, 'launch', 'moveit_rviz.launch.py')
        )
    )

    # 8️⃣ 启动 MoveGroup
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_panda, 'launch', 'move_group.launch.py')
        )
    )

    # 当 joint_state_broadcaster 结束加载后再加载 panda_arm_controller
    delayed_arm_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[panda_arm_spawner],
        )
    )

    delayed_hand_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=panda_arm_spawner,
            on_exit=[panda_hand_spawner],
        )
    )

    # 构建 LaunchDescription
    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        ros2_control_node,
        spawn_entity,
        bridge,
        joint_state_broadcaster_spawner,
        delayed_arm_spawner,
        delayed_hand_spawner,
        move_group,
        moveit_rviz
    ])

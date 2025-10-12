import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get the package directory
    panda_configure_dir = get_package_share_directory('panda_configure')
    
    # Gazebo world file
    world_file = os.path.join(panda_configure_dir, 'worlds', 'empty.world')
    
    # URDF file for Gazebo
    urdf_file = os.path.join(panda_configure_dir, 'config', 'panda.gazebo.xacro')
    
    # MoveIt configuration
    moveit_config = MoveItConfigsBuilder("panda", package_name="panda_configure").to_moveit_configs()
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': moveit_config.robot_description,
            'use_sim_time': True
        }],
        arguments=[urdf_file]
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'panda',
            '-file', urdf_file,
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'rate': 30
        }]
    )
    
    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'panda_link0']
    )
    
    # RViz
    rviz_config_file = os.path.join(panda_configure_dir, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ]
    )
    
    # MoveGroup
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(panda_configure_dir, 'launch', 'move_group.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_publisher,
        static_tf,
        rviz_node,
        move_group,
    ])

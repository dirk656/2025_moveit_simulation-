#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class PandaTestNode(Node):
    def __init__(self):
        super().__init__('panda_test_node')
        
        # Create publisher for arm trajectory
        self.arm_publisher = self.create_publisher(
            JointTrajectory,
            '/panda_arm_controller/joint_trajectory',
            10
        )
        
        # Create publisher for hand trajectory
        self.hand_publisher = self.create_publisher(
            JointTrajectory,
            '/panda_hand_controller/joint_trajectory', 
            10
        )
        
        # Wait a bit for controllers to be ready
        time.sleep(2.0)
        
        # Test arm movement
        self.test_arm_movement()
        
        # Test hand movement
        self.test_hand_movement()
    
    def test_arm_movement(self):
        """Test moving the arm to a simple position"""
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # Simple pose
        point.time_from_start.sec = 5
        
        trajectory.points.append(point)
        
        self.get_logger().info('Sending arm trajectory...')
        self.arm_publisher.publish(trajectory)
        time.sleep(6.0)  # Wait for movement to complete
    
    def test_hand_movement(self):
        """Test opening and closing the hand"""
        trajectory = JointTrajectory()
        trajectory.joint_names = ['panda_finger_joint1']
        
        # Open hand
        point_open = JointTrajectoryPoint()
        point_open.positions = [0.04]  # Open position
        point_open.time_from_start.sec = 2
        trajectory.points = [point_open]
        
        self.get_logger().info('Opening hand...')
        self.hand_publisher.publish(trajectory)
        time.sleep(3.0)
        
        # Close hand
        point_close = JointTrajectoryPoint()
        point_close.positions = [0.0]  # Closed position
        point_close.time_from_start.sec = 2
        trajectory.points = [point_close]
        
        self.get_logger().info('Closing hand...')
        self.hand_publisher.publish(trajectory)
        time.sleep(3.0)

def main():
    rclpy.init()
    node = PandaTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

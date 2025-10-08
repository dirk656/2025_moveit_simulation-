#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

def test_comparison_system():
    rclpy.init()
    
    # 创建节点来测试服务
    node = Node('test_comparison_client')
    
    # 创建服务客户端
    client = node.create_client(Trigger, '/compare_planners')
    
    # 等待服务可用
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error('Comparison service not available')
        return False
    
    # 发送请求
    request = Trigger.Request()
    future = client.call_async(request)
    
    # 等待响应
    rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
    
    if future.done():
        try:
            response = future.result()
            if response.success:
                node.get_logger().info('Comparison test completed successfully!')
                node.get_logger().info(f'Message: {response.message}')
                return True
            else:
                node.get_logger().error(f'Comparison test failed: {response.message}')
                return False
        except Exception as e:
            node.get_logger().error(f'Service call failed: {str(e)}')
            return False
    else:
        node.get_logger().error('Service call timed out')
        return False

if __name__ == '__main__':
    success = test_comparison_system()
    exit(0 if success else 1)

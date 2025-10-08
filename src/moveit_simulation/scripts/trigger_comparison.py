#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

class TriggerComparison(Node):
    def __init__(self):
        super().__init__('trigger_comparison')
        
        # 创建服务客户端
        self.client = self.create_client(Trigger, '/comparison_node/compare_planners')
        
        # 等待服务可用
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for comparison service...')
        
        self.get_logger().info('Comparison trigger node started')
        
        # 延迟后自动触发对比
        self.create_timer(5.0, self.trigger_comparison)
    
    def trigger_comparison(self):
        self.get_logger().info('Triggering planner comparison...')
        
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.comparison_callback)
    
    def comparison_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Comparison completed successfully:')
                self.get_logger().info(response.message)
            else:
                self.get_logger().error('Comparison failed: ' + response.message)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))

def main():
    rclpy.init()
    node = TriggerComparison()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

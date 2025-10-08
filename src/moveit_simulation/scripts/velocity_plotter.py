#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np
import threading
import time

class VelocityPlotter(Node):
    def __init__(self):
        super().__init__('velocity_plotter')
        
        # 订阅速度数据
        self.velocity_sub = self.create_subscription(
            Float64MultiArray,
            '/velocity_comparison_data',
            self.velocity_callback,
            10
        )
        
        # 数据存储
        self.timestamps = []
        self.rh_velocities = []
        self.traditional_velocities = []
        
        # 绘图设置
        self.fig, self.ax = plt.subplots(figsize=(12, 6))
        plt.ion()  # 交互模式
        plt.show(block=False)
        
        # 设置图表
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Joint Velocity (rad/s)')
        self.ax.set_title('Planner Velocity Comparison - Panda Joint 7')
        self.ax.legend()
        self.ax.grid(True, alpha=0.3)
        
        self.get_logger().info('Velocity plotter started')
        
        # 上次更新时间
        self.last_update_time = time.time()
        
    def velocity_callback(self, msg):
        if len(msg.data) >= 3:
            timestamp = msg.data[0]
            rh_velocity = msg.data[1]
            traditional_velocity = msg.data[2]
            
            # 确保所有数组长度一致
            self.timestamps.append(timestamp)
            self.rh_velocities.append(rh_velocity)
            self.traditional_velocities.append(traditional_velocity)
            
            # 保持最近100个数据点
            max_points = 100
            if len(self.timestamps) > max_points:
                self.timestamps = self.timestamps[-max_points:]
                self.rh_velocities = self.rh_velocities[-max_points:]
                self.traditional_velocities = self.traditional_velocities[-max_points:]
            
            # 限制更新频率（每0.5秒更新一次）
            current_time = time.time()
            if current_time - self.last_update_time > 0.5:
                self.update_plot()
                self.last_update_time = current_time
    
    def update_plot(self):
        if len(self.timestamps) > 0:
            # 转换为相对时间
            if len(self.timestamps) > 1:
                relative_times = [t - self.timestamps[0] for t in self.timestamps]
            else:
                relative_times = [0.0]
            
            # 清除并重新绘制
            self.ax.clear()
            
            # 确保数组长度一致
            min_len = min(len(relative_times), len(self.rh_velocities), len(self.traditional_velocities))
            if min_len > 0:
                relative_times = relative_times[:min_len]
                rh_velocities = self.rh_velocities[:min_len]
                traditional_velocities = self.traditional_velocities[:min_len]
                
                # 绘制速度曲线
                self.ax.plot(relative_times, rh_velocities, 'r-', label='Receding Horizon', linewidth=2)
                self.ax.plot(relative_times, traditional_velocities, 'b-', label='Traditional', linewidth=2)
                
                # 设置图表
                self.ax.set_xlabel('Time (s)')
                self.ax.set_ylabel('Joint Velocity (rad/s)')
                self.ax.set_title('Planner Velocity Comparison - Panda Joint 7')
                self.ax.legend()
                self.ax.grid(True, alpha=0.3)
                
                # 自动调整坐标轴范围
                if len(relative_times) > 1:
                    self.ax.set_xlim(0, max(relative_times))
                    
                    all_velocities = rh_velocities + traditional_velocities
                    if all_velocities:
                        max_vel = max(abs(v) for v in all_velocities)
                        self.ax.set_ylim(-max_vel * 1.1, max_vel * 1.1)
                
                # 刷新显示
                plt.draw()
                plt.pause(0.01)

def main():
    rclpy.init()
    
    # 创建节点
    plotter = VelocityPlotter()
    
    # 在单独的线程中运行ROS节点
    def spin_node():
        rclpy.spin(plotter)
    
    spin_thread = threading.Thread(target=spin_node)
    spin_thread.daemon = True
    spin_thread.start()
    
    try:
        # 保持主线程运行
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        plotter.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()

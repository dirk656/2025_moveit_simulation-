# 滚动时域算法与传统算法速度对比系统

## 系统概述

本系统实现了滚动时域规划算法（Receding Horizon Planning）与传统线性插值算法的速度对比功能，能够在rqt中实时显示最末端关节的速度曲线。

## 系统组件

### 1. 核心节点
- **comparison_node**: 对比节点，提供两种规划器的性能对比服务
- **velocity_plotter.py**: 速度绘图器，在rqt中实时显示速度曲线
- **trigger_comparison.py**: 自动触发对比的服务调用脚本

### 2. 启动文件
- **comparison_demo.launch.py**: 完整的对比演示启动文件

## 使用方法

### 方法1：使用完整演示启动文件（推荐）
```bash
# 在终端1：启动对比演示
ros2 launch moveit_simulation comparison_demo.launch.py

# 在终端2：启动速度绘图器
ros2 run moveit_simulation velocity_plotter.py

# 在终端3：触发对比
ros2 run moveit_simulation trigger_comparison.py
```

### 方法2：手动启动各个组件
```bash
# 终端1：启动MoveIt和对比节点
ros2 launch panda_arm_moveit_description demo.launch.py &
ros2 run moveit_simulation comparison_node

# 终端2：启动速度绘图器
ros2 run moveit_simulation velocity_plotter.py

# 终端3：触发对比
ros2 service call /compare_planners std_srvs/srv/Trigger
```

### 方法3：使用测试脚本
```bash
# 测试整个系统
ros2 run moveit_simulation test_comparison_system.py
```

## 功能特性

### 1. 算法对比
- **滚动时域规划器**: 基于时间最优TVP的轨迹生成
- **传统规划器**: 简单的线性插值方法

### 2. 实时可视化
- 在rqt中实时显示两种算法的速度曲线对比
- 显示最末端关节（panda_joint7）的速度变化
- 自动更新图表，支持实时数据流

### 3. 性能指标
- 轨迹执行时间对比
- 速度平滑度分析
- 最大速度限制检查

## 数据流

1. **服务调用**: `/compare_planners` 服务触发对比
2. **轨迹规划**: 两种算法分别生成轨迹
3. **速度发布**: 发布到 `/velocity_comparison_data` 话题
4. **可视化**: velocity_plotter.py 订阅并显示速度数据

## 输出结果

- **终端输出**: 显示两种算法的执行时间和性能指标
- **rqt图表**: 显示实时速度曲线对比
- **服务响应**: 返回对比结果和状态信息

## 依赖要求

- ROS2 Humble
- MoveIt2
- matplotlib
- rclpy
- std_msgs
- visualization_msgs

## 故障排除

1. **服务不可用**: 确保comparison_node已启动
2. **图表不显示**: 检查velocity_plotter.py是否正常运行
3. **轨迹规划失败**: 检查MoveIt配置和机器人状态

## 扩展功能

系统支持进一步扩展：
- 添加更多规划器算法对比
- 增加更多性能指标（加速度、加加速度等）
- 支持多关节速度对比
- 添加轨迹跟踪误差分析

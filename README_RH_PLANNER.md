# 滚动时域规划器 (Receding Horizon Planner) - RViz集成指南

## 概述

本包提供了一个滚动时域规划器(Receding Horizon Planner)的MoveIt集成，可以在RViz中实时演示你的算法。该规划器实现了时间最优轨迹规划，特别适用于机械臂的平滑运动控制。

## 主要功能

1. **时间最优轨迹规划** - 基于TVP(梯形速度曲线)的时间最优规划
2. **滚动时域控制** - 支持动态路径更新
3. **MoveIt集成** - 与MoveIt框架无缝集成
4. **RViz可视化** - 在RViz中实时显示规划结果

## 文件结构

```
moveit_simulation/
├── include/moveit_simulation/
│   └── rh_planner.hpp          # 滚动时域规划器头文件
├── src/
│   ├── rh_planner.cpp          # 滚动时域规划器实现
│   ├── rh_moveit_integration.cpp # MoveIt集成节点
│   ├── planner.cpp             # 基础规划器节点
│   └── test.cpp                # 测试节点
├── launch/
│   ├── rh_planner_demo.launch.py # 滚动时域规划器演示启动文件
│   └── test.launch.py          # 测试启动文件
└── README_RH_PLANNER.md        # 本文件
```

## 使用方法

### 1. 构建项目

```bash
cd /home/feng/2025simulation_moveit
colcon build --packages-select moveit_simulation
source install/setup.bash
```

### 2. 启动滚动时域规划器演示

```bash
ros2 launch moveit_simulation rh_planner_demo.launch.py
```

### 3. 在RViz中使用

#### 如何启动算法

**完整启动流程：**

1. **构建项目**：
   ```bash
   cd /home/feng/2025simulation_moveit
   colcon build --packages-select moveit_simulation
   source install/setup.bash
   ```

2. **启动演示系统**：
   ```bash
   ros2 launch moveit_simulation rh_planner_demo.launch.py
   ```
   这将启动：
   - MoveGroup节点
   - RViz可视化界面
   - 控制器管理器
   - 滚动时域规划器节点 (rh_moveit_integration)

3. **验证系统状态**：
   ```bash
   # 检查节点是否运行
   ros2 node list | grep rh_moveit_integration
   
   # 检查服务是否可用
   ros2 service list | grep rh
   ```

#### 如何观察算法

**在RViz中观察算法效果：**

1. **设置目标姿态**：
   - **方法1（交互式）**：在RViz中，使用"Interactive Markers"拖拽机械臂末端到目标位置
   - **方法2（命令行）**：通过ROS话题发布目标姿态：
     ```bash
     ros2 topic pub /rh_moveit_integration/target_pose geometry_msgs/msg/PoseStamped "{
       header: {frame_id: 'panda_link0'},
       pose: {
         position: {x: 0.3, y: 0.2, z: 0.5},
         orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
       }
     }" --once
     ```

2. **触发滚动时域规划**：
   ```bash
   ros2 service call /rh_moveit_integration/plan_with_rh std_srvs/srv/Trigger
   ```

3. **观察算法效果**：

   **在RViz中观察：**
   - **轨迹可视化**：观察机械臂的规划轨迹（橙色线条）
   - **实时运动**：观看机械臂按照滚动时域规划生成的平滑轨迹运动
   - **交互标记**：目标位置显示为绿色标记
   - **碰撞检测**：观察规划器如何避开障碍物（如果配置了碰撞物体）

   **在终端中观察：**
   - **规划统计**：查看滚动时域规划器的计算时间和段数
   - **算法输出**：观察时间最优TVP规划的具体参数
   - **性能指标**：监控轨迹执行时间和精度

   **算法特点观察：**
   - **时间最优性**：观察机械臂以最快速度到达目标位置
   - **平滑运动**：注意轨迹的连续性和平滑性
   - **关节同步**：观察多个关节的协调运动
   - **滚动时域**：尝试动态更新目标，观察实时重新规划

4. **测试不同场景**：
   - 尝试不同的目标位置
   - 测试接近关节极限的位置
   - 观察算法如何处理复杂路径
   - 验证滚动时域控制的实时响应

#### 快速测试命令

```bash
# 一次性测试：设置目标并触发规划
source install/setup.bash
ros2 topic pub /rh_moveit_integration/target_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'panda_link0'},
  pose: {
    position: {x: 0.3, y: 0.2, z: 0.5},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}" --once && ros2 service call /rh_moveit_integration/plan_with_rh std_srvs/srv/Trigger
```

### 4. 手动测试

```bash
# 启动基础演示
ros2 launch moveit_simulation test.launch.py

# 在另一个终端中运行规划器节点
ros2 run moveit_simulation rh_moveit_integration
```

## 算法特点

### 时间最优TVP规划
- 支持梯形和三角形速度曲线
- 考虑关节速度和加速度限制
- 自动选择最优速度曲线类型

### 滚动时域控制
- 支持动态路径更新
- 实时轨迹重新规划
- 方向切换点检测

### 关节同步
- 多关节运动同步
- 通过同步时间确保协调运动
- 考虑不同关节的运动阶段

## 配置参数

### Panda机械臂默认参数
```cpp
// 最大速度 (rad/s)
std::vector<double> max_velocities = {
    2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100
};

// 最大加速度 (rad/s²)
std::vector<double> max_accelerations = {
    15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0
};
```

### 控制参数
- 控制周期: 1ms
- 路径插值点数: 10
- 容差: 1e-6

## 服务接口

### 规划服务
- **服务名称**: `/rh_moveit_integration/plan_with_rh`
- **服务类型**: `std_srvs/srv/Trigger`
- **功能**: 触发滚动时域规划

### 话题接口

#### 订阅
- **话题**: `/rh_moveit_integration/target_pose`
- **类型**: `geometry_msgs/msg/Pose`
- **功能**: 接收目标姿态

#### 发布
- **话题**: `/joint_trajectory_controller/joint_trajectory`
- **类型**: `trajectory_msgs/msg/JointTrajectory`
- **功能**: 发布关节轨迹

## 性能统计

规划器提供以下统计信息：
- 总计算时间 (ms)
- 计算段数
- 平均计算时间 (ms)

## 故障排除

### 常见问题

1. **规划失败**:
   - 检查目标姿态是否可达
   - 验证逆运动学计算
   - 检查关节限制

2. **轨迹执行失败**:
   - 确认控制器运行正常
   - 检查轨迹消息格式
   - 验证时间戳

3. **编译错误**:
   - 确保所有依赖包已安装
   - 检查CMakeLists.txt配置
   - 验证头文件包含路径

### 调试技巧

- 启用详细日志输出
- 使用RViz可视化规划结果
- 检查服务调用响应

## 扩展开发

要扩展或修改算法：

1. **修改规划参数**: 在`rh_planner.cpp`中调整机器人参数
2. **添加新功能**: 在`rh_planner.hpp`中定义新接口
3. **集成新控制器**: 修改`rh_moveit_integration.cpp`中的发布器

## 参考

- [MoveIt2文档](https://moveit.ros.org/)
- [ROS2文档](https://docs.ros.org/)
- 时间最优轨迹规划相关论文

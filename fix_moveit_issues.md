# MoveIt 机械臂运动问题修复方案

## 问题分析

从日志中识别的主要问题：

### 1. 控制器轨迹时间戳问题
```
[ERROR] [1760098515.525004459] [panda_arm_controller]: Time between points 0 and 1 is not strictly increasing, it is 0.000000 and 0.000000 respectively
```

### 2. 运动学求解器配置问题
```
[ERROR] [1760098501.208960536] [moveit_kinematics_base.kinematics_base]: Group 'panda_hand' is not a chain
[ERROR] [1760098501.208963357] [kinematics_plugin_loader]: Kinematics solver of type 'kdl_kinematics_plugin/KDLKinematicsPlugin' could not be initialized for group 'panda_hand'
```

### 3. Pilz规划器配置问题
```
[ERROR] [1760098538.624400640] [moveit.pilz_industrial_motion_planner]: No ContextLoader for planner_id '' found. Planning not possible.
```

### 4. 关节加速度限制缺失
```
[WARN] [1760098546.528240975] [moveit_trajectory_processing.time_optimal_trajectory_generation]: Joint acceleration limits are not defined. Using the default 1 rad/s^2.
```

## 已实施的修复

### 1. 运动学求解器配置优化
- 增加求解器超时时间从 0.005s 到 0.05s
- 增加求解器尝试次数从 3 到 5
- 文件：`kinematics.yaml`

### 2. 关节限制配置优化
- 增加默认速度和加速度缩放因子从 0.1 到 0.3
- 为所有关节添加加速度限制 (3.0 rad/s²)
- 文件：`joint_limits.yaml`

### 3. 控制器配置优化
- 添加关节增益控制参数 (PID)
- 添加轨迹约束参数 (goal_time, stopped_velocity_tolerance)
- 明确指定插值方法为 splines
- 文件：`ros2_controllers.yaml`

## 推荐的启动命令

使用以下命令启动，优先使用OMPL规划器：

```bash
source install/setup.bash && ros2 launch moveit_rh_planner demo_rh_planner.launch.py
```

## 故障排除

### 如果仍然遇到问题：

1. **检查规划器选择**：
   - 在RViz中确保选择正确的规划器（推荐OMPL）
   - 避免使用Pilz规划器，除非明确配置

2. **验证控制器状态**：
   ```bash
   ros2 control list_controllers
   ros2 control list_hardware_interfaces
   ```

3. **检查轨迹执行**：
   ```bash
   ros2 topic echo /joint_states
   ros2 topic echo /panda_arm_controller/feedback
   ```

4. **重置系统**：
   ```bash
   # 停止所有节点
   Ctrl+C
   # 重新构建
   colcon build --packages-select moveit_rh_planner panda_arm_moveit_description
   # 重新启动
   source install/setup.bash && ros2 launch moveit_rh_planner demo_rh_planner.launch.py
   ```

## 成功指标

- OMPL规划器应该能够成功规划并执行轨迹
- CHOMP规划器可能仍然有轨迹时间戳问题，但OMPL应该可靠工作
- 机械臂应该能够平滑移动到目标位置
- 控制器应该报告"Goal reached, success!"

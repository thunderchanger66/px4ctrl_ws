# 开源 EGO 轨迹接入 PX4 的实现方式调研

调研日期：2026-11-03

## 结论先行

EGO-Planner 本身只负责局部轨迹规划。它输出的是连续 B-spline，`traj_server` 按当前时间采样为：

```text
position, velocity, acceleration, yaw, yaw_dot
```

公开工程通常有两条控制链路：

```text
EGO PositionCommand -> PX4 TrajectorySetpoint -> PX4 位置/速度控制器 -> 姿态/推力
```

或：

```text
EGO PositionCommand -> 外部位置/速度/加速度控制器 -> AttitudeTarget / vehicle_attitude_setpoint
                       -> PX4 姿态环/角速度环 -> 电机
```

两条链路不能同时启用。第一条让 PX4 负责位置环，第二条绕过 PX4 位置环，由外部控制器负责从轨迹误差计算推力和姿态。

## 1. EGO 官方输出是什么

EGO-Planner 上游仓库：

- [EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)
- [EGO-Swarm / ROS2 branch](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version)

上游 `traj_server` 接收 B-spline 后，不把控制点发送给飞控，而是周期性计算：

```cpp
pos = position_traj(t);
vel = velocity_traj(t);
acc = acceleration_traj(t);
yaw = calculate_yaw(t, pos);
yawdot = ...;
```

再发布 `quadrotor_msgs/PositionCommand`。因此 PX4 接入层应消费采样后的命令，而不是自己重新解析 B-spline。

你当前的 `px4ctrl_ws/src/ego/ego_planner/src/traj_server.cpp` 采用的就是这个接口。

## 2. 方案 A：转成 PX4 `TrajectorySetpoint`，使用 PX4 位置环

一个较新的 ROS2 开源工程：

- [N1up666/px4-ros2-ego-planner-navigation](https://github.com/N1up666/px4-ros2-ego-planner-navigation)
- 关键文件：[ego_px4_setpoint_bridge.cpp](https://github.com/N1up666/px4-ros2-ego-planner-navigation/blob/main/ros2_ws/src/ego_px4_bridge/src/ego_px4_setpoint_bridge.cpp)
- 架构说明：[architecture.md](https://github.com/N1up666/px4-ros2-ego-planner-navigation/blob/main/docs/architecture.md)

其核心逻辑是直接把 `PositionCommand` 转成 PX4 `TrajectorySetpoint`：

```cpp
out.position = {msg->position.y, msg->position.x, -msg->position.z};
out.velocity = {msg->velocity.y, msg->velocity.x, -msg->velocity.z};
out.acceleration = {msg->acceleration.y, msg->acceleration.x, -msg->acceleration.z};
out.yaw = pi/2 - msg->yaw;
out.yawspeed = NaN;
```

也就是 ENU/lidar 风格坐标到 PX4 NED 的变换：

```text
x_NED = y_ENU
y_NED = x_ENU
z_NED = -z_ENU
```

随后持续发布：

```text
/fmu/in/offboard_control_mode
/fmu/in/trajectory_setpoint
```

PX4 自己执行位置控制、速度控制、加速度到姿态/推力的转换。PX4 官方 ROS2 示例也是这种思路：

- [PX4 offboard_control.cpp](https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp)
- [PX4 PositionControl.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/PositionControl/PositionControl.cpp)

PX4 内部大致计算：

```text
vel_sp = Kp_position * (pos_sp - pos) + vel_ff
acc_sp = Kp_velocity * (vel_sp - vel) + acc_ff
body_z = normalize([-acc_x, -acc_y, g - acc_z])
thrust = function(acc_z, hover_thrust, body_z.z)
```

这条路径的优点是接口简单、与 PX4 参数和 failsafe 集成好；缺点是轨迹跟踪效果依赖 PX4 的 `MPC_XY_P`、`MPC_XY_VEL_P_ACC`、`MPC_THR_HOVER`、倾角和推力限制，外部 EGO 的高阶轨迹不一定被完整利用。

## 3. 方案 B：EGO `PositionCommand` 进入外部几何控制器

较典型的真实飞行实现：

- [xdu-aero-association/ego_planner_for_px4_with_vins_fusion](https://github.com/xdu-aero-association/ego_planner_for_px4_with_vins_fusion)
- [real_flight_modules/px4ctrl](https://github.com/xdu-aero-association/ego_planner_for_px4_with_vins_fusion/tree/main/real_flight_modules/px4ctrl)
- 同类仓库：[gwxie2/ego-px4ctrl-new](https://github.com/gwxie2/ego-px4ctrl-new)

其 `controller.cpp` 使用 EGO 的位置、速度、加速度和 yaw：

```text
e_p = p_des - p
v_cmd = v_des + Kp * e_p
e_v = v_cmd - v
u_v = Kv * e_v + integral(e_v)
F_des = m * (u_v + Ka * a_des + [0, 0, g])
```

然后：

```text
z_b_des = F_des / ||F_des||
y_c_des = [-sin(yaw), cos(yaw), 0]
x_b_des = normalize(y_c_des × z_b_des)
y_b_des = z_b_des × x_b_des
R_des = [x_b_des y_b_des z_b_des]
```

最后通过 MAVROS `mavros_msgs/AttitudeTarget` 发布姿态四元数和归一化推力。该仓库还包含：

- `AUTO_HOVER -> CMD_CTRL` 状态机；
- 丢失轨迹命令时回悬停；
- 悬停推力在线估计；
- 位置、速度、加速度和倾角约束；
- 姿态反馈与期望姿态的 yaw 对齐。

这条路径和当前 `px4ctrl_ws/src/px4ctrl/src/px4ctrl_node.cpp` 的方向相同：当前实现将 EGO 采样命令转换为 NED，加 PD/前馈计算期望加速度，再按 PX4 `ControlMath::bodyzToAttitude()` 生成 `VehicleAttitudeSetpoint`。

## 4. 方案 C：只把位置和 yaw 转发给 PX4/MAVROS

较老的实验仓库：

- [Quaddict/ego_planner_px4](https://github.com/Quaddict/ego_planner_px4)
- 关键文件：[ego_to_mavros.cpp](https://github.com/Quaddict/ego_planner_px4/blob/master/offboard_px4/src/ego_to_mavros.cpp)

该实现只读取 EGO 的 `position` 和 `yaw`，通过 MAVROS `PositionTarget` 发送位置设定点，忽略 EGO 的 velocity/acceleration：

```text
EGO position/yaw -> MAVROS PositionTarget -> PX4 位置环
```

它适合验证“能否让飞机到达目标”，不适合充分利用 EGO 的平滑速度和加速度轨迹，也不适合作为高动态跟踪参考。

## 5. 三种方案的差异

| 方案 | PX4 接口 | 谁负责位置控制 | 是否使用 EGO 速度/加速度 | 适用情况 |
|---|---|---|---|---|
| A | `TrajectorySetpoint` | PX4 | 是 | 推荐的标准 PX4 Offboard 路径 |
| B | `VehicleAttitudeSetpoint` / `AttitudeTarget` | 外部 px4ctrl | 是 | 需要精确几何跟踪、绕过 PX4 位置环 |
| C | `PositionTarget` 位置模式 | PX4 | 否或很少 | 简单验证、旧 MAVROS 工程 |

## 6. 对你当前工程的判断

你当前已经选择了方案 B，而不是网上最常见的方案 A：

```text
EGO PositionCommand
 -> px4ctrl 几何控制器
 -> /fmu/in/vehicle_attitude_setpoint_v1
 -> PX4 姿态控制器
```

这条链路原理上是成立的，但它比 `TrajectorySetpoint` 路径多承担了四项工作：

1. planner/lidar 到 PX4/NED 的原点和旋转对齐；
2. 位置误差、速度误差和加速度前馈的增益设计；
3. body-z、yaw、倾角和推力饱和；
4. 外部视觉估计、PX4 local reset、命令中断和起飞交接。

当前源码已按 PX4 官方 `ControlMath` 的符号和几何关系重写 body-z 与推力计算，并使用 PX4 local position/velocity 作为闭环反馈。这样与公开 px4ctrl 的数学路径一致，但输出接口适配 PX4 ROS2 的 `VehicleAttitudeSetpoint`。

## 7. 最值得借鉴的实现细节

### 从方案 A 借鉴

- 位置、速度、加速度都明确转换到 NED；
- `OffboardControlMode` 和实际 setpoint 持续发布；
- EGO odometry 使用 PX4 local position/velocity 反向转换到规划坐标系；
- 轨迹命令超时必须保持合法 setpoint，不能继续使用过期参考。

### 从方案 B 借鉴

- 位置误差先转换为速度修正，再与 EGO 速度前馈相加；
- 速度积分只在误差小、没有明显饱和时更新；
- 悬停推力应标定或在线估计，不能照搬其他机型参数；
- `AUTO_HOVER` 和 `CMD_CTRL` 明确分开，命令中断回悬停。

### 不建议直接照搬

- 旧 MAVROS 示例中的固定二进制 `type_mask`；
- 只发送位置而丢弃 EGO velocity/acceleration；
- 把 EGO ENU yaw 直接当 PX4 NED yaw；
- 同时发布 `TrajectorySetpoint` 和 `VehicleAttitudeSetpoint` 争夺不同控制层。

## 8. 对当前控制器的推荐路线

如果目标是“稳定、易迁移、先完成任务”，优先使用方案 A：让 PX4 位置环跟踪 EGO 的 position/velocity/acceleration。

如果目标是“研究高精度轨迹控制”，继续使用方案 B，但应把它视为独立的外部控制器，必须单独标定：

```text
1. PX4 local position/velocity 延迟与 reset
2. hover_thrust
3. Kp/Kv/Ka
4. 最大倾角和最大推力
5. 姿态环跟踪延迟
6. 轨迹替换时的起点连续性
```

当前工程已经完成方案 B 的核心几何公式修正。下一步如果要进一步贴近公开真实飞行实现，最有价值的改动是加入明确的速度积分抗饱和和轨迹替换连续性检查，而不是继续增加急停条件。

## 参考链接

- [EGO-Planner upstream](https://github.com/ZJU-FAST-Lab/ego-planner)
- [EGO-Swarm ROS2](https://github.com/ZJU-FAST-Lab/ego-planner-swarm/tree/ros2_version)
- [PX4 ROS2 Offboard example](https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp)
- [PX4 PositionControl](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mc_pos_control/PositionControl/PositionControl.cpp)
- [N1up666 PX4 + EGO ROS2 bridge](https://github.com/N1up666/px4-ros2-ego-planner-navigation)
- [xdu-aero-association px4ctrl](https://github.com/xdu-aero-association/ego_planner_for_px4_with_vins_fusion/tree/main/real_flight_modules/px4ctrl)
- [Quaddict EGO to MAVROS](https://github.com/Quaddict/ego_planner_px4/blob/master/offboard_px4/src/ego_to_mavros.cpp)

# PX4Ctrl 调研与落地说明

## 参考实现

- [PX4 Offboard mode](https://docs.px4.io/main/en/flight_modes/offboard.html)
- [PX4 external vision](https://docs.px4.io/main/en/ros/external_position_estimation.html)
- [PX4 ROS 2 navigation interface](https://docs.px4.io/main/en/ros2/px4_ros2_navigation_interface.html)
- [Fast-Drone-250 px4ctrl](https://github.com/ZJU-FAST-Lab/Fast-Drone-250/tree/master/src/realflight_modules/px4ctrl)

PX4 的 Offboard 模式要求持续发布 `OffboardControlMode`。本工作空间选择
`attitude=true`，因此必须同时发布 `VehicleAttitudeSetpoint`，并且 PX4 的姿态估计
和外部视觉估计必须有效。多旋翼的 `thrust_body[2]` 是负的归一化推力需求。

Fast-Drone-250 的实现采用位置误差、速度误差和期望加速度组成的级联控制律，并且
使用独立状态机管理 `MANUAL -> TAKEOFF -> HOVER -> CMD -> LAND`。它不会在第一条
轨迹命令到达时直接把轨迹交给飞控，也不会把固定的百分比推力当成通用标定值。

## 当前实现

`px4ctrl_node` 现在按下面的状态工作：

1. 等待 FAST-LIO、PX4 local position 和第一条有效 EGO 命令。
2. 进入 Offboard 前先持续发送姿态设定点，保持当前 NED 水平位置。
3. 自动起飞到 `auto_takeoff_altitude_m`，默认 0.50 m。
4. 速度低于 `hover_velocity_tolerance_mps` 并保持 `hover_settle_sec` 后才进入轨迹跟踪。
5. 轨迹命令超时后回到当前位置悬停，不继续使用旧目标。
6. 外部视觉、PX4 local position 或规划器报告异常时锁存故障并请求降落。

默认控制循环为 100 Hz。保持点的坐标在 PX4 NED 中保存，送入几何控制律前会经过
`rotation_` 的逆变换，避免起飞阶段重复做 lidar/NED 坐标变换。

## 真正飞行前的验证顺序

### 1. 外部视觉静止测试

不启动控制器，只运行 PX4、Gazebo、FAST-LIO、PGO 和 bridge，静止记录至少 60 秒：

```bash
ros2 topic echo /fastlio2/lio_odom
ros2 topic echo /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position_v1
```

位置应连续，速度接近 0，不能出现跳变、时间戳倒退或 `dead_reckoning: true`。bridge
检测到物理上不可能的跳变会停止外部视觉并发布 `/planning/emergency_stop`。

### 2. PX4 参数和时序

按实际 PX4 固件版本检查，而不是盲目复制数值：

```text
EKF2_EV_CTRL   外部视觉融合的维度
EKF2_HGT_REF   高度参考
EKF2_EV_DELAY  传感器捕获到 EKF 融合的延迟
EKF2_EV_POS_X/Y/Z 传感器相对机体的位置
MPC_THR_HOVER  PX4 内部位置控制使用的悬停推力
COM_OF_LOSS_T  Offboard 链路丢失等待时间
COM_OBL_ACT    Offboard 链路丢失动作
```

外部视觉时间戳必须尽量使用传感器采样时间；仅使用 ROS 消息到达时间会让
`EKF2_EV_DELAY` 随系统负载变化。

### 3. 只做起飞和悬停

使用当前参数启动 `px4ctrl`，发送一个目标后只观察 `/px4ctrl/status` 和 PX4 local
position。必须依次看到：

```text
state_takeoff -> state_hover -> state_tracking
```

如果起飞过程中位置估计无效、速度异常或推力不足，停止调参并先修复估计器、
坐标变换或 `hover_thrust`。不要用提高最大推力掩盖估计器问题。

### 4. 小范围水平移动

先测试 0.5 m 以内、低速和低加速度目标，再逐步增加距离。每次测试都记录 PX4
`.ulg`，检查位置误差、速度、姿态、推力和 estimator innovation。只有静止、起飞、
水平移动都连续通过后，才测试障碍环境。

## 推力标定

`hover_thrust` 是当前机体和 PX4 输入接口对应的归一化悬停推力，不是通用常数。
在仿真中可从 `MPC_THR_HOVER` 和实际悬停高度误差校准；实机必须在受控环境中逐步
增加推力并记录起飞临界值。标定改变机体、电池、桨叶或 PX4 airframe 后必须重新做。

## 不能宣称的结论

通过编译、进入 Offboard 或短暂解锁，都不能证明可以真实飞行。当前工作空间仍需
在目标 PX4 固件、目标机体和经过标定的 FAST-LIO/外部视觉链上完成上述验证，并以
PX4 `.ulg` 日志确认估计器创新和控制误差后，才可以进入真实飞行测试。

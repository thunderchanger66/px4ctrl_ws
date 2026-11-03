# PX4Ctrl 控制链路与数学约定

## 1. 数据流

```text
FAST-LIO/PGO (/fastlio2/lio_odom, lidar/body)
        ├── fastlio2_px4_odom_bridge -> PX4 VehicleOdometry (NED/FRD)
        └── EGO planner -> PositionCommand (lidar frame)

PX4 VehicleLocalPosition (NED) + PositionCommand
        -> PX4Ctrl 几何控制器
        -> VehicleAttitudeSetpoint (q_d, thrust_body[2])
        -> PX4 姿态环/角速度环/控制分配 -> 电机
```

对齐完成后，PX4 local position 是控制器唯一的反馈状态。FAST-LIO 的作用是给 EGO 提供规划状态，并通过 bridge 给 PX4 EKF 提供外部视觉；控制器不再把 FAST-LIO 的位置和 PX4 的速度混成一个误差。

## 2. 坐标系

EGO 使用 lidar 世界系，PX4 使用 NED 世界系。当前模型的固定旋转为：

```text
R_ned_lidar = [[0, 1, 0],
               [1, 0, 0],
               [0, 0,-1]]
```

因此 `p_ned = R_ned_lidar p_lidar + t`，速度和加速度只乘旋转，不能加平移。`t` 在系统静止时由 PX4 local position 与 FAST-LIO position 求得；PX4 发生正常 local-frame reset 时，`delta_xy/delta_z` 同步加到该平移和所有保存的 NED 目标上。

FAST-LIO 的 `nav_msgs/Odometry.twist` 是 child/body 系速度。EGO 和 odom bridge 都使用发布的姿态将它旋转到 lidar/NED 世界系后再使用。

## 3. 几何控制公式

期望加速度由 EGO 前馈和 PD 误差组成：

```text
a = a_ff + Kp (p_d - p) + Kv (v_d - v)
```

先限制 `||a|| <= a_max`。NED 中重力向下，因此 PX4 需要的世界系推力方向对应：

```text
f = a - [0, 0, g]
b3_raw = -f / ||f||
```

`b3` 是机体 z 轴在 NED 中的方向。控制器按 PX4 `ControlMath::limitTilt()` 将 `b3` 与 `[0,0,1]` 的夹角限制到 `max_tilt`，再按 PX4 `bodyzToAttitude()` 的构造方法生成姿态：

```text
y_C = [-sin(yaw), cos(yaw), 0]
b1 = normalize(y_C × b3)
b2 = b3 × b1
R = [b1 b2 b3]
q_d = quat(R)
```

倾角限制后，推力幅值必须用受限 `b3` 的垂向投影计算，否则会破坏垂向加速度：

```text
F_down = max(0.1, g - a_z)
T_norm = hover_thrust * F_down / (g * b3.z)
```

最后将 `T_norm` 限制在 `[min_thrust,max_thrust]`，发布 `thrust_body = [0,0,-T_norm]`。这与 PX4 `PositionControl::_accelerationControl()` 输出的负 body-z 推力符号一致。PX4 仍负责姿态误差到角速度和电机控制，PX4Ctrl 不绕过姿态环。

## 4. 状态机边界

- 起飞阶段只跟踪 PX4 local NED 的起飞目标。
- 起飞稳定后，EGO 轨迹通过平滑交接混入，避免轨迹已经前进而控制器刚解锁时产生参考跳变。
- 规划命令短时失效时保持当前位置和零速度；EGO 规划失败时发布静止 hold B-spline。
- 低于起飞原点安全高度的恢复轨迹不会直接驱动下降；如果当前已高于安全线，则保持当前高度。
- PX4 local position 的连续性、有效性和 dead-reckoning 状态由 PX4 反馈判断。正常小 reset 做坐标补偿，异常大 reset 才进入故障流程。

## 5. 本轮验证

构建：

```bash
source /opt/ros/jazzy/setup.bash
source ~/GRA_1/ws_livox/install/setup.bash
cd ~/GRA_1/px4ctrl_ws
colcon build --packages-select px4ctrl --symlink-install --executor sequential
```

`2026-11-03_mathfix_1m.csv`：单向 1 m 目标，稳定段 RMS 三维误差约 0.051 m，末端约 0.054 m；PX4 保持 Offboard、armed、local position valid、非 dead-reckoning。

`2026-11-03_mathfix_roundtrip.csv`：`(0,0,0.5) -> (4,-4,0.5) -> (0,0,0.5)`。到达 `(4,-4)` 后约 6--7 cm RMS；返程期间 PX4 始终保持 Offboard、armed、非 dead-reckoning。150 s 记录结束时 EGO 仍处于返程轨迹中，末端误差约 0.35 m，属于轨迹尚未完成，不是控制器失控。该任务包含多次 EGO 重规划，不能用单一终点误差代替完整任务成功判定。

每一轮实验前后应停止 PX4、Gazebo、FAST-LIO/PGO、EGO、PX4Ctrl 和 Micro XRCE Agent。`bringup.launch.py` 中 agent 已取消 respawn，避免停止后残留进程占用 UDP 8888 干扰下一轮。

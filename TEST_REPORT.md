# PX4Ctrl 跟踪 EGO 轨迹测试报告

日期：2026-11-03  
结果：当前 PX4 SITL 配置下，PX4Ctrl 完成了 EGO 生成的 1 m 水平位移和 0.5 m 高度目标跟踪。

## 目标与边界

验证复制到 `px4ctrl_ws` 的 EGO Planner 输出能否经 PX4Ctrl 转换为姿态与推力设定点，并由 PX4 SITL 实际跟踪。测试使用 X500 + gimbal + MID-360 Gazebo 模型、FAST-LIO2/PGO 外部视觉定位与 PX4 Offboard。此结果仅代表本次 SITL 配置；没有测试障碍物重规划、长距离任务或真实飞行。

## 本次修复

- PX4Ctrl 此前长期处于起飞状态：载荷模型没有达到 0.5 m 的完整起飞目标，状态机因此一直没有把控制权交给 EGO 轨迹。
- 起飞门控现在要求至少经过 3 秒、速度稳定，并且达到完整目标高度或相对起飞点爬升至少 0.25 m，之后才转入悬停和轨迹跟踪。最低离地高度可在 `src/px4ctrl/config/px4ctrl_sitl.yaml` 的 `minimum_takeoff_altitude_m` 配置。
- 将此仿真机体实测悬停推力设为 `hover_thrust: 0.864`。这是当前 X500 + gimbal + MID-360 模型的 SITL 标定值，不应直接用于其他机体或实机。
- 使用 `/fmu/in/vehicle_attitude_setpoint_v1`，并以 PX4 local position 有效性、时效和 dead-reckoning 状态作为输出门控。

## 实验步骤

每次运行前先停止上一轮所有终端中的 launch，并确认 PX4、Gazebo、Micro XRCE-DDS、FAST-LIO/PGO、EGO 与 PX4Ctrl 进程已退出。本次有效跟踪运行从空进程状态开始。

1. 终端 A 启动 PX4/Gazebo。将仿真 launch 文件复制到 PX4 launch 目录后启动：

   ```bash
   cd ~/PX4-Autopilot/launch
   source ~/GRA_1/ws_livox/install/setup.bash
   source ~/GRA_1/mid360_ws/install/setup.bash
   cp /home/thunder/GRA_1/mid360_ws/src/mid360_gazebo_harmonic/launch/mid360_posix_sitl.launch.py .
   ros2 launch ./mid360_posix_sitl.launch.py gui:=false use_nvidia_gpu:=false
   ```

2. 终端 B 启动 FAST-LIO2/PGO：

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/GRA_1/ws_livox/install/setup.bash
   source ~/GRA_1/fastlio2_pgo_ws/install/setup.bash
   ros2 launch pgo pgo_launch.py
   ```

3. 终端 C 启动 PX4Ctrl bringup。该 launch 启动 EGO、轨迹服务器、FAST-LIO 到 PX4 的里程计桥、PX4Ctrl 与 Micro XRCE-DDS Agent：

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/GRA_1/ws_livox/install/setup.bash
   source ~/GRA_1/px4ctrl_ws/install/setup.bash
   ros2 launch px4ctrl bringup.launch.py start_rviz:=false
   ```

4. 终端 D 启动 CSV 记录，再发送 `(x,y,z,yaw)=(1.0,0.0,0.5,0)` 的 EGO 目标：

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/GRA_1/ws_livox/install/setup.bash
   source ~/GRA_1/px4ctrl_ws/install/setup.bash
   ros2 run px4ctrl track_eval --csv /home/thunder/GRA_1/px4ctrl_ws/test_results/2026-11-03_track_1m_hover864.csv --duration 70
   ```

   记录器运行期间，在另一终端执行：

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/GRA_1/ws_livox/install/setup.bash
   source ~/GRA_1/px4ctrl_ws/install/setup.bash
   ros2 run px4ctrl manual_goal 1.0 0.0 0.5 0 --wait-timeout 30 --publish-count 10
   ```

5. 记录结束后停止本轮所有 launch，确认进程清空，再开始下一轮。正常停止时先停止 PX4Ctrl bringup；若 Offboard setpoint 中断，PX4 会按配置进入 Offboard-loss 保护并尝试降落，因此应观察该动作完成后再关闭 PX4/Gazebo。

## 实测结果

原始采样文件：[`test_results/2026-11-03_track_1m_hover864.csv`](test_results/2026-11-03_track_1m_hover864.csv)。采样周期约 50 ms。EGO 命令在 `lidar` 坐标系，PX4 位置在 NED；比较前按配置的旋转变换还原坐标：`lidar=[px4_y_ned, px4_x_ned, -px4_z_ned]`。

- 记录中有 1,075 个样本同时满足 PX4 armed、Offboard、定位有效且非 dead-reckoning，覆盖约 53.7 秒。
- 末端 EGO 命令为 `(1.002, 0.001, 0.502) m`；PX4 NED 位置为 `(-0.024, 0.972, -0.502) m`，换算后末端三维位置误差 **0.040 m**。
- 最后 20 秒稳定段的三维位置误差 RMS **0.039 m**，最大 **0.044 m**；末端速度各轴绝对值均低于 `0.004 m/s`。
- 全段误差峰值约 **1.097 m**，发生在起飞/交接期间：EGO 命令已移动到约 0.98 m，而 PX4Ctrl 仍在按状态机完成离地阶段。它不代表稳定跟踪段误差，但说明起飞与轨迹启动之间存在可见的交接延迟；后续可优化为起飞完成后重新生成/触发 EGO 轨迹。
- 记录期间 PX4 保持 Offboard（`nav_state=14`）、armed（`arming_state=2`）；位置有效，未进入 dead reckoning。

停机时，先前被停止的 PX4Ctrl 不再提供 Offboard setpoint，PX4 日志出现 `invalid setpoints`/`blind land`。随后 PX4/Gazebo 与 FAST-LIO/PGO launch 均以 SIGINT 正常退出。该保护动作出现在测试结束后的控制器停机阶段，不属于有效飞行区间内的轨迹跟踪失败。

## 结论与限制

本轮完成了“EGO 位置/速度/加速度命令 -> PX4Ctrl 几何姿态与推力 -> PX4 Offboard”的闭环 SITL 跟踪，目标终点误差约 4 cm。此前不能进入跟踪状态的起飞门控问题已修复，且本次数据证明控制器已经进入并完成目标跟踪。

这不是避障验收：没有加入新障碍物，也没有验证 EGO 重规划后的连续轨迹交接。PX4Ctrl 直接输出姿态/推力并绕过 PX4 位置环，安全余度和实机性能仍需在 SITL 做多方向、多速度、扰动和障碍测试，并以 PX4 ULog 分析姿态响应、推力饱和及估计器创新后，才能考虑受控实机验证。

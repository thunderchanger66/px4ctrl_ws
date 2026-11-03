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

## 2026-11-03 坐标对齐回归复测

本次代码复核发现原控制器把 planner `lidar` 位置仅做旋转，而 EV 桥接器会额外将 FAST-LIO 原点平移对齐到 PX4 local NED。现已在控制器启动时以静止状态计算并应用同一类平移；PX4 local position reset 在已解锁时触发锁存故障，未解锁时重新对齐。另修复 FAST-LIO 超时后控制器仍持续刷新 Offboard 模式、却不再发送姿态设定点的失效路径：已解锁时现请求 PX4 降落。

验证状态：`colcon build --packages-up-to px4ctrl` 和修改后的 `colcon build --packages-select px4ctrl --symlink-install` 均通过。首次回归误用了 PX4 launch 目录里的旧副本，该副本没有设置 `EKF2_DELAY_MAX=800`；失败数据保存在 `test_results/2026-11-03_frame_alignment_validation.csv`。第二次使用 `mid360_ws` 最新 launch，PX4 日志确认 `EKF2_EV_CTRL=7`、`EKF2_DELAY_MAX=800`、`EKF2_HGT_REF=3`。EV 位置/速度创新未被拒绝，但 ULog 中 EV 数据在 PX4 仿真时间约 54.7 s 后停止；约 55.1 s 后 `cs_ev_pos/vel/hgt` 变为 false，local position 进入 dead-reckoning，控制器状态停在 `waiting_for_healthy_px4_local_position`，未解锁且未执行目标。该数据保存在 `test_results/2026-11-03_frame_alignment_success.csv`，文件名仅表示使用新 launch 的复测轮次，不代表飞行通过。

因此，本次源码改动已构建，但其飞行闭环回归尚未通过。下一步应先追踪 EV 发布为何在仿真 55 s 左右停止（检查 FAST-LIO/PGO 输出与桥接器锁存日志），再重新验证坐标偏移非零时的目标跟踪。两轮 PGO 正常退出时执行了其配置的地图自动保存；请留意 `fastlio2_pgo_ws` 的地图文件是否需要恢复到测试前版本。

## 2026-11-03 控制交接与时钟映射更新

- EGO 到 PX4 的 `hover -> tracking` 交接增加 0.75 s smoothstep 参考混合，避免起飞期间 EGO 轨迹已经前进、控制器解锁跟踪时参考位置突然跳变。
- 控制器现在将 `PositionCommand.yaw_dot` 按 planner-to-NED 旋转映射后，作为限幅后的 PX4 姿态 yaw-rate feedforward；最大值默认 1.5 rad/s。
- FAST-LIO bridge 不再假设 ROS 仿真时间与 PX4 时间的偏移恒定。它根据持续到达的 PX4 local-position 时间与 ROS `/clock` 估计时钟速率和偏移，并按 `max_sample_age_sec: 0.75` 丢弃陈旧 odometry。这里保留原始采样年龄供 EKF2 使用，没有把旧样本改写成当前样本。
- 本轮同步观察到 FAST-LIO 样本年龄约 40-110 ms，PX4 收到的 `timestamp_sample` 滞后约 50-120 ms；重启前固定偏移映射出现约 18 s 偏差。更新后 PX4 local position 在采样窗口内为 valid 且未 dead-reckoning。
- `fastlio2_px4_odom_bridge` 与 `px4ctrl` 目标包构建通过。全依赖构建另外尝试一次，但在未修改的 `px4_msgs` ament Python symlink 步骤因 build 目录已有同名目录失败；目标包构建不受影响。
- 本轮没有发送目标或解锁。PX4 `pre_flight_checks_pass` 保持 false，因此不能把本轮记为新的飞行回归通过。PX4 源码的 `mc_att_control_main.cpp` 也确认姿态输入只接受严格递增的 timestamp；控制器继续使用最新 PX4 输出时间戳。官方 Offboard 机制说明见 [PX4 Offboard 文档](https://docs.px4.io/main/en/flight_modes/offboard.html)。

## 2026-11-03 动态轨迹回归

一轮 0.5 m 目标保持测试中，PX4 保持 armed/Offboard 30 s，EGO 命令持续有效，末端三维误差约 5.2 cm。随后发送相邻 1 m 目标验证运动段时，PX4 的 `xy/z_reset_counter` 在起飞/估计器初始化过程中变化，旧控制器把任意计数器变化当作故障并停止姿态设定点，PX4 随后进入 `AUTO_LAND`。该行为不是目标跟踪误差，而是 reset 处理过于严格。

现已按 PX4 `mc_pos_control` 的处理方式修改：使用 `delta_xy`/`delta_z` 同步补偿控制器保存的 NED 目标；仅当 reset 位移超过 `max_ekf_reset_delta_m` 或偏航重置超过 `max_ekf_reset_heading_rad` 时才锁存安全故障。修改后需重新启动完整 SITL，再重复动态轨迹测试。

重新启动全新 PX4/Gazebo、FAST-LIO 和 PX4Ctrl 后，动态回归成功。测试文件为
`test_results/2026-11-03_ego_attitude_fresh_1m_motion.csv`：记录 30 s、221 个样本；其中 180 个样本处于 armed + Offboard，全部样本 `xy_valid=true` 且 `dead_reckoning=false`。EGO 轨迹从起飞点平滑推进到约 `(1.0, 0.0, 0.5)` m，末端 PX4 NED 位置约 `(0.006, 0.951, -0.506)` m，对应 lidar 位置误差 **4.4 cm**。全段 RMS 包含起飞阶段为约 0.493 m，最大误差约 0.926 m；稳定跟踪末端误差约 4-5 cm。测试结束后已停止 PX4、Gazebo、FAST-LIO 和 PX4Ctrl 进程。

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

## 2026-11-03 大范围往返复现与修复

我另外做了连续任务 `0,0,0.5 -> 4,-4,0.5 -> 0,0,0.5`，原始记录为 [`test_results/2026-11-03_roundtrip_4m_before_fix.csv`](test_results/2026-11-03_roundtrip_4m_before_fix.csv)。到达 `(4,-4,0.5)` 时 PX4 仍为 armed/Offboard，定位有效；故障出现在返航重规划阶段：

- EGO 日志报告 `Rejecting final trajectory: sample 5.333 s is inside inflated occupancy`，说明返航候选轨迹未通过膨胀地图碰撞检查。
- 规划器随后反复报告 `Trajectory finished 5.62 m from goal; replanning from odometry`，但失败分支仍可能让轨迹服务器继续发布旧的、未通过验证的轨迹。
- PX4Ctrl 没有主动触发 emergency stop；它忠实跟踪了 EGO 发布的命令。记录中 `cmd_z_lidar` 在返航失败轨迹期间降到约 `0.08 m`，所以无人机表现为下降。这不是 PX4 推力不足造成的。

对应修复如下：

- EGO 的重规划失败、返航失败和碰撞替代轨迹失败分支现在调用 `publishHoldTrajectory()`，用当前位置生成静止 B-spline，清除目标并进入 `WAIT_TARGET`，不会再执行失败或残留轨迹。
- FAST-LIO/PGO bridge 对不超过 `1.50 m` 的有界回环位姿校正不再直接急停，而是递增 `VehicleOdometry.reset_counter`、将该采样速度置零并重新建立连续性；超过该阈值或超出飞行边界仍触发急停。配置项为 `max_pose_reset_jump_m`。
- PX4Ctrl 增加 `minimum_tracking_altitude_m: 0.25`。EGO 短时发布低于安全离地高度的恢复轨迹时，控制器将其钳制到起飞原点上方 0.25 m，并清除向下速度/加速度，避免规划失败导致下降。

修复后工作区重新构建通过 9 个包。连续往返复现的最后记录显示 PX4 一直保持 `nav_state=14`、armed、`dead_reckoning=0`，未收到 bridge emergency stop；返航候选轨迹仍被 EGO 的膨胀地图拒绝时，系统现在应保持当前位置等待新目标，而不是下降执行旧轨迹。修复后的正式往返验收需要在更新后的二进制上重新启动一轮，并检查 CSV 中返航失败后的 `cmd_z_lidar` 是否保持当前位置高度。

本次新门控修改后已完成参数加载/节点启动冒烟测试和完整编译；尚未重新跑完 150 秒往返任务，因此最低高度钳制的最终 CSV 证据待下一轮长时仿真确认。已有修复后 CSV 证明 EGO safe hold 能保持高度，新增门控进一步覆盖其短暂低轨迹窗口。

最后又修正了最低高度门控方向：若飞行器已经高于安全线，低高度命令现在保持当前高度；只有低于安全线时才爬升到安全线。修改后 `px4ctrl` 目标再次构建通过，参数加载冒烟测试正常，所有仿真进程已清理。

## 2026-11-03 姿态变化与自动降落复现

PX4Ctrl 的输出链路是：EGO `/drone_0_planning/pos_cmd` 的位置、速度、加速度和 yaw 经过坐标变换与 PD 几何控制，生成期望机体四元数和 `thrust_body[2]`，发布到 `/fmu/in/vehicle_attitude_setpoint_v1`。PX4 仍执行姿态环和控制分配，但不再执行 PX4 的位置环。

测试得到两种不同的降落原因：

- 单独发送 90 deg 航向目标时，车辆保持 armed/Offboard，未进入 dead-reckoning，说明大 yaw 变化本身不会触发降落。原始 1 m 多场景测试中，PX4 local position 报告 `delta_xy=[-0.244,-1.125] m`，旧的 `max_ekf_reset_delta_m=1.0` 将其判为故障，PX4Ctrl 发布 `NAV_LAND`。现已把 SITL 阈值设为 3.0 m，并按 PX4 `MulticopterPositionControl` 同步补偿 `delta_xy`、`delta_z` 和 `delta_heading`；另外加入 1 s 连续故障确认，避免短暂估计器间隙立即降落。
- 使用 PGO 时，回环优化会改变 FAST-LIO 世界位姿，可能产生较大的外部视觉 reset。该场景应作为定位压力测试，不能与纯控制器跟踪误差混合统计。
- 不含 PGO 的测试中，EGO 对回到 `(0,0,0.5)` 的目标报 `occupied or out-of-map`，随后进入 `EMERGENCY_STOP`。EGO 的 `/planning/emergency_stop` 被 PX4Ctrl 订阅后会主动发送 `NAV_LAND`，所以这类降落来自规划器安全策略，不是姿态四元数或推力计算。

复现数据：`test_results/2026-11-03_turn_90.csv`、`test_results/2026-11-03_multiscene.csv`、`test_results/2026-11-03_lio_multiscene.csv`。修复后的配置和代码已通过 `colcon build --packages-select px4ctrl --symlink-install --executor sequential` 与 `git diff --check`。

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

## 2026-11-03 控制数学重构回归

本轮按 PX4 `PositionControl::_accelerationControl()` 和
`ControlMath::thrustToAttitude()` 重新核对并改写了 PX4Ctrl 的几何控制段：

- 加速度误差仍使用 EGO 前馈加位置/速度 PD，但控制反馈统一为 PX4 `VehicleLocalPosition` 的 NED 状态；FAST-LIO 只用于规划输入和 PX4 外部视觉估计。
- 期望推力先按 `f=a-[0,0,g]` 转换为 body-z，再用 PX4 同样的 `limitTilt()` 和 `bodyzToAttitude()` 构造四元数。
- 倾角限制后按受限 body-z 的垂向投影重新计算 collective thrust，避免原先“姿态已经限倾、推力仍按限倾前力模长计算”的不一致。
- `bringup.launch.py` 的 Micro XRCE Agent 取消 respawn，停止实验后不会再自动占用 8888 端口。

构建通过：`px4ctrl` 目标包 9 个依赖组件无错误，`git diff --check` 和 Python 语法检查通过。

单向 1 m 实测记录为 `test_results/2026-11-03_mathfix_1m.csv`。稳定段 RMS 三维误差约 5.1 cm，末端误差约 5.4 cm；PX4 全程保持 Offboard、armed、local position valid、非 dead-reckoning。

4 m 往返实测记录为 `test_results/2026-11-03_mathfix_roundtrip.csv`。到达 `(4,-4,0.5)` 后稳定误差约 6--7 cm。返程阶段 PX4 仍保持 armed/Offboard/定位有效，记录结束时 EGO 仍在返程重规划轨迹中，末端误差约 0.35 m，未出现控制链路失控或自动降落。该轮持续 150 s，返程尚未在记录窗口内完全结束，因此不能把它记为“返回原点成功”。

控制链路、坐标变换和公式的完整说明见 [`CONTROL_CHAIN.md`](CONTROL_CHAIN.md)。

公开 EGO/PX4 接入方式的源码调研见 [`ONLINE_EGO_PX4_REFERENCE.md`](ONLINE_EGO_PX4_REFERENCE.md)。

一轮 0.5 m 目标保持测试中，PX4 保持 armed/Offboard 30 s，EGO 命令持续有效，末端三维误差约 5.2 cm。随后发送相邻 1 m 目标验证运动段时，PX4 的 `xy/z_reset_counter` 在起飞/估计器初始化过程中变化，旧控制器把任意计数器变化当作故障并停止姿态设定点，PX4 随后进入 `AUTO_LAND`。该行为不是目标跟踪误差，而是 reset 处理过于严格。

现已按 PX4 `mc_pos_control` 的处理方式修改：使用 `delta_xy`/`delta_z` 同步补偿控制器保存的 NED 目标；仅当 reset 位移超过 `max_ekf_reset_delta_m` 或偏航重置超过 `max_ekf_reset_heading_rad` 时才锁存安全故障。修改后需重新启动完整 SITL，再重复动态轨迹测试。

重新启动全新 PX4/Gazebo、FAST-LIO 和 PX4Ctrl 后，动态回归成功。测试文件为
`test_results/2026-11-03_ego_attitude_fresh_1m_motion.csv`：记录 30 s、221 个样本；其中 180 个样本处于 armed + Offboard，全部样本 `xy_valid=true` 且 `dead_reckoning=false`。EGO 轨迹从起飞点平滑推进到约 `(1.0, 0.0, 0.5)` m，末端 PX4 NED 位置约 `(0.006, 0.951, -0.506)` m，对应 lidar 位置误差 **4.4 cm**。全段 RMS 包含起飞阶段为约 0.493 m，最大误差约 0.926 m；稳定跟踪末端误差约 4-5 cm。测试结束后已停止 PX4、Gazebo、FAST-LIO 和 PX4Ctrl 进程。
## 2026-11-03 坐标系交接修复后的独立回归

本轮先停止了所有旧的 PX4/Gazebo、Micro XRCE-DDS、FAST-LIO/PGO、EGO 和 PX4Ctrl 进程，再只启动一套链路。启动初期 FAST-LIO 尚未形成有效地图约束，bridge 正确等待 PX4 local position，控制器保持未解锁；定位恢复后 bridge 完成原点对齐，才发送目标并开始记录。

本轮修复了 PX4Ctrl 重规划平滑的坐标系顺序：EGO 的旧/新参考现在先在 `lidar` 坐标系中 smoothstep 混合，再统一转换到 PX4 local NED。旧实现会把 planner-frame 参考和 NED 参考直接混合，在当前轴交换矩阵下产生虚假的交接跳变。

记录文件：[`test_results/2026-11-03_px4ctrl_postblend_1m.csv`](test_results/2026-11-03_px4ctrl_postblend_1m.csv)。目标为 `(1.0, 0.0, 0.5)` m，记录 45 s，共 862 个样本；所有样本 `xy_valid=1`、`z_valid=1`、`dead_reckoning=0`。PX4 在有效飞行段进入并保持 `nav_state=14`（Offboard）和 `arming_state=2`（armed）。

- 最后 20 s 三维位置误差 RMS：**0.048 m**；最大 **0.054 m**。
- 最后 10 s 三维位置误差 RMS：**0.047 m**；末端误差 **0.049 m**。
- 末端 PX4 NED 位置约 `(-0.024, 0.957, -0.491)` m，按 `lidar=[y_ned,x_ned,-z_ned]` 换算后与目标一致。
- 全段 RMS 为 **0.233 m**，峰值发生在起飞和控制交接阶段；稳定跟踪段保持厘米级误差。

本轮验证的是单向跟踪和控制交接，不是障碍物重规划验收；返程和动态障碍仍需在地图有效、目标可达的前提下单独统计。

最后又收紧了姿态设定点时间戳处理：当 PX4 输出时间戳在相邻回调中重复时，只增加 1 微秒维持严格递增，不再每次增加 1 毫秒，从而避免输出流暂时变慢时设定点时钟向未来漂移。修改后 `px4ctrl` 重新构建通过，Python 语法检查和 `git diff --check` 通过。

## 2026-11-05 auto-yaw 与 Z 轴独立复测

从完全停止状态分别重启 SITL 全链路，对 `(0,0,0.5)` 运行 `--auto-yaw` 与固定 90 度航向测试。两轮均保持 armed、Offboard、定位有效，飞行记录中 emergency 计数为 0。auto-yaw 最后 15 秒 Z 误差 RMS 为 1.2 cm，固定航向为 7.3 cm；两轮早期误差主要来自起飞和交接窗口。当前测试未复现 auto-yaw 导致下坠，因此没有依据修改 yaw/Z 控制律。数据与分析见 [`test_results/2026-11-05_auto_yaw_zaxis_analysis.md`](test_results/2026-11-05_auto_yaw_zaxis_analysis.md)，原始记录为同目录下的 `2026-11-05_auto_yaw_zaxis.csv` 和 `2026-11-05_fixed_yaw_zaxis.csv`。旧 `yaw_takeoff_retest.csv` 仍显示异常垂向状态，但其 ULog 未能与 CSV 时间对齐；需用同时包含 EGO 加速度、PX4 实际姿态和估计器 reset 的采集再现，不能据此认定 auto-yaw 是根因。

## 2026-11-06 auto-yaw 返航修复与完整复测

针对 `(0,0,0.5) -> (4,-4,0.5) -> (0,0,0.5)` 的 auto-yaw 返航异常，PX4Ctrl 原先只有 yaw 角速度上限，路径方向反转时角速度会直接跳到 `1.5 rad/s`。已增加 yaw 角加速度限制，并将角速度和姿态四元数按同一平滑斜坡输出；默认最大 yaw 角速度 `0.6 rad/s`、最大角加速度 `0.8 rad/s^2`。采集器补记 EGO 加速度/yaw_dot、PX4 local acceleration、heading/reset counters 和 yaw-rate feed-forward。

在每次全部关闭后分别完成 auto-yaw 和固定 `-83 deg` 航向 round-trip。两轮都到达两个端点并稳定，保持正常 tracking，emergency 为 0，PX4 高度 reset 计数保持不变。auto-yaw 返航端点为 `(0.038,-0.012,0.501)`、速度 `0.026 m/s`；返航最大 `|vz|=0.113 m/s`，无 thrust saturation。固定航向返航端点为 `(-0.011,-0.017,0.492)`、速度 `0.008 m/s`。详细诊断、启动顺序和原始 CSV 见 [`test_results/2026-11-06_auto_yaw_return_analysis.md`](test_results/2026-11-06_auto_yaw_return_analysis.md)。结论仅适用于当前 SITL 全链路，不作为真实飞行调参完成的依据。

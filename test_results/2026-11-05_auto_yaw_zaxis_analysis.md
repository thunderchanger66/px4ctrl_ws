# PX4Ctrl Auto-Yaw and Z-Axis Regression

Date: 2026-11-05

## Objective

Check whether `manual_goal --auto-yaw` causes vertical trajectory loss, compare it with a fixed 90 degree yaw goal, and inspect the EGO-to-PX4Ctrl attitude/thrust path. Each trial used a fresh PX4/Gazebo, FAST-LIO/PGO, EGO, Micro XRCE-DDS Agent, and PX4Ctrl startup.

## Procedure

Before each trial, the prior PX4/Gazebo, Micro XRCE-DDS Agent, FAST-LIO/PGO, EGO, trajectory server, and PX4Ctrl processes were stopped and the process list checked. The test stack was then started in this order:

1. PX4/Gazebo from `~/PX4-Autopilot/launch` with the MID-360 SITL launch, `gui:=false` and `use_nvidia_gpu:=false`.
2. FAST-LIO/PGO with `ros2 launch pgo pgo_launch.py`.
3. EGO, trajectory server, odometry bridge, Micro XRCE-DDS Agent, and PX4Ctrl with `ros2 launch px4ctrl bringup.launch.py start_rviz:=false`.
4. `track_eval` for 55 seconds, followed by one goal at `(0.0, -0.0, 0.5)`.

Trial commands:

```bash
ros2 run px4ctrl manual_goal 0.0 -0.0 0.5 --auto-yaw --wait-timeout 30
ros2 run px4ctrl manual_goal 0.0 -0.0 0.5 90 --wait-timeout 30
```

Each CSV was recorded before the goal was sent. PX4Ctrl was then stopped, PX4 was allowed to enter its configured Offboard-loss protection, and all launch processes were shut down before the next trial. The PX4 logs report landing detection during shutdown; this shutdown behavior is outside the in-flight tracking windows.

## Results

| Trial | Samples | Armed + Offboard | Emergency count | Z error RMS, 30-45 s | Max Z error, 30-45 s | End state |
|---|---:|---|---:|---:|---:|---|
| Auto-yaw | 952 | Yes | 0 | 0.0115 m | 0.0169 m | Armed, Offboard, valid local position |
| Fixed 90 deg | 905 | Yes | 0 | 0.0730 m | 0.0779 m | Armed, Offboard, valid local position |

The Z error is computed as `cmd_z_lidar + px4_z_ned`, using the configured planner-to-NED mapping where planner Z-up maps to negative PX4 NED Z. The 30-45 second window excludes takeoff and handoff. The auto-yaw run reached a stable final altitude: command `0.5009 m`, PX4 NED Z `-0.4815 m`, and vertical speed `-0.0068 m/s`. The fixed-yaw run ended at command `0.5008 m`, PX4 NED Z `-0.4331 m`, and vertical speed `0.0031 m/s`, showing a steady height bias in this single run.

During the first 15 seconds, both trials include takeoff and planner handoff. Auto-yaw Z RMS/max error was `0.245/0.486 m`; fixed-yaw was `0.164/0.459 m`. Auto-yaw's 15-30 second window had `0.055 m` RMS error and peak thrust magnitude `0.896`; the fixed-yaw run had `0.150 m` RMS error, `0.491 m` peak error, and repeatedly reached the configured thrust limit `0.95`. Neither trial triggered `/planning/emergency_stop`, lost local-position validity, entered dead reckoning, or left Offboard while the test recorder was active.

Raw data:

- [`2026-11-05_auto_yaw_zaxis.csv`](2026-11-05_auto_yaw_zaxis.csv)
- [`2026-11-05_fixed_yaw_zaxis.csv`](2026-11-05_fixed_yaw_zaxis.csv)

## Code-path findings

- `manual_goal --auto-yaw` clears `/px4ctrl/manual_yaw`; it does not change Z position, velocity, acceleration, thrust, or the PX4 control mode. EGO's `PositionCommand.yaw` is then used by PX4Ctrl.
- PX4Ctrl transforms EGO position, velocity, and acceleration into PX4 local NED, computes acceleration as feed-forward plus position/velocity feedback and a bounded integral term, projects it into thrust and tilt limits, then creates the attitude quaternion and collective thrust. Yaw only selects the horizontal attitude heading; the vertical thrust calculation uses the NED vertical acceleration and body-Z projection.
- The current vector signs match PX4's NED convention: `force = acceleration - [0, 0, g]`, body-Z follows `-force`, and PX4 receives negative `thrust_body[2]`. The hover setpoint observed in steady tracking is approximately `-0.864`.
- The current clean auto-yaw test changes EGO yaw to `pi` as the vertical trajectory begins. With the configured lidar-to-NED rotation, this maps to the PX4 launch heading, so this same-XY test does not command a large physical turn. It validates the auto-yaw path and vertical hold, but not a high-rate turning trajectory.
- `track_eval` records EGO yaw, not the controller's resolved yaw target. Therefore its `cmd_yaw_lidar` column does not prove the exact manual-yaw setpoint during the fixed-yaw trial.

## Interpretation and next diagnostic

The reported drop is not reproduced by these two fresh-start trials. The current evidence does not justify changing PX4Ctrl's Z gains or yaw controller. The older `2026-11-05_yaw_takeoff_retest.csv` includes a planner emergency and a subsequent command-Z collapse, but its PX4 state and extreme vertical-speed samples do not establish that yaw caused the initial vertical event. The ULog available during this investigation could not be aligned confidently with that CSV, so it is not used as causal evidence.

For a decisive reproduction of the original symptom, record a trajectory with simultaneous horizontal translation and a substantial heading change, plus EGO acceleration, PX4 attitude setpoint and actual attitude, local-position acceleration, estimator reset counters, and PX4 command acknowledgements on a common timestamp. The existing `track_eval` CSV lacks several of those signals. Until that capture exists, the sound engineering result is that auto-yaw has passed same-XY vertical-hold SITL, while the older failure remains unexplained rather than fixed by an unverified controller change.

## Build and cleanup

`colcon build --packages-up-to px4ctrl --symlink-install --executor sequential` completed successfully (9 packages). Both test stacks were stopped; no PX4, Gazebo, XRCE Agent, FAST-LIO/PGO, EGO, trajectory-server, or PX4Ctrl process remained at cleanup.

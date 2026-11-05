# Auto-Yaw Return Regression and Verification

Date: 2026-11-06

## Symptom and diagnosis

The reported failure occurs on the return leg, when EGO reverses its path heading by nearly 180 degrees. In the captured failure (`2026-11-05_auto_yaw_roundtrip.csv`), PX4Ctrl allowed a yaw setpoint rate of `1.5 rad/s` immediately after the EGO yaw command changed. During that same interval, the altitude reference remained near `0.5 m`, but thrust reached its `0.95` limit and PX4 vertical velocity/position departed sharply. EGO then replaced the moving reference with a hold, the emergency counter increased, and the controller latched after local-position health was lost. The emergency/landing sequence was downstream of the initial vertical deviation.

The existing code limited yaw rate but had no yaw angular-acceleration limit. A path-heading reversal could therefore request the full yaw rate in one control update. The logs support this as the trigger for the observed coupled transient; the repeatable clean test below verifies that smoothing this demand removes the transient. The previous single-goal altitude tests did not exercise this reversal and therefore did not cover the reported case.

## Change

PX4Ctrl now slews the yaw-rate setpoint with configurable limits on both yaw rate and yaw angular acceleration. The same profiled yaw rate is sent in `VehicleAttitudeSetpoint.yaw_sp_move_rate`, keeping PX4's attitude reference model consistent with the moving quaternion target. Defaults are `0.6 rad/s` and `0.8 rad/s^2`; the SITL YAML uses the same values. Position, velocity, acceleration, thrust, and safety logic are unchanged.

`track_eval` now records EGO acceleration and yaw rate, PX4 local acceleration and heading, estimator reset counters, and PX4 yaw-rate feed-forward. Those signals distinguish trajectory demand, controller response, and estimator resets in future runs.

## Clean test procedure

Each experiment began with no PX4, Gazebo, XRCE Agent, FAST-LIO/PGO, EGO, bridge, PX4Ctrl, or recorder processes. The stack was started in this order:

1. Source ROS Jazzy, `ws_livox`, and `mid360_ws`; launch MID-360 PX4/Gazebo SITL with GUI and NVIDIA rendering disabled.
2. Source `fastlio2_pgo_ws`; launch `pgo pgo_launch.py`.
3. Source `px4ctrl_ws`; launch `px4ctrl bringup.launch.py start_rviz:=false` (this starts Agent, EGO, trajectory server, odometry bridge, and PX4Ctrl).
4. Start `track_eval`, send the outbound goal once, and wait for FAST-LIO position within `0.2 m` and speed below `0.15 m/s` for two seconds.
5. Send the return goal once, apply the same arrival/stability test at the origin, stop the recorder, then stop all launch processes before the next experiment.

The auto-yaw commands were:

```bash
ros2 run px4ctrl manual_goal 4.0 -4.0 0.5 --auto-yaw --wait-timeout 45
ros2 run px4ctrl manual_goal 0.0 -0.0 0.5 --auto-yaw --wait-timeout 45
```

The fixed-heading control used `-83` degrees for both goals. Each goal was published once; the earlier exploratory fixed-heading run that used `--publish-count 10` is invalid for comparison because repeated goals caused EGO replanning and was not used in the results below.

## Results

| Trial | Outbound endpoint | Return endpoint | Return max Z error | Return max `|vz|` | Max yaw-rate FF | Thrust at limit | EGO emergency | PX4 Z reset |
|---|---|---|---:|---:|---:|---:|---:|---|
| Auto-yaw, smoothed | `(3.999,-3.975,0.508)`, `0.043 m/s` | `(0.038,-0.012,0.501)`, `0.026 m/s` | `0.163 m` | `0.113 m/s` | `0.60 rad/s` | 0 samples | 0; counter stayed 2 |
| Fixed `-83 deg` | `(3.958,-4.029,0.489)`, `0.052 m/s` | `(-0.011,-0.017,0.492)`, `0.008 m/s` | `0.084 m` | `0.075 m/s` | `0 rad/s` | 0 samples | 0; counter stayed 2 |

Both trials reached both goals, remained in `tracking_attitude_thrust_tracking`, and recorded zero planner emergencies. From the start of the return transition through the end of each capture, auto-yaw had `0.051 m` RMS Z error and fixed heading had `0.059 m` RMS Z error. Neither run showed the previous vertical transient or a PX4 estimator reset. Each test was fully shut down before the other was started, and no related process remained afterward.

Raw captures:

- `2026-11-06_auto_yaw_smooth_roundtrip.csv`
- `2026-11-06_fixed_yaw_smooth_roundtrip.csv`
- Original failure: `2026-11-05_auto_yaw_roundtrip.csv`

## Verification and limits

`colcon build --packages-select px4ctrl --symlink-install --executor sequential` passed after the controller change. `track_eval.py` passed Python syntax validation. The two clean end-to-end SITL trials verify the reported 4 m return path with the current EGO, FAST-LIO/PGO, odometry bridge, PX4, and PX4Ctrl launch chain. This is SITL evidence; it does not establish real-flight tuning or dynamic-obstacle performance.

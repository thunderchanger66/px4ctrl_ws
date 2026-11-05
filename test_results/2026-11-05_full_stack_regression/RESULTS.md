# Full-Stack SITL Regression

Date: 2026-11-05

## Scope

One clean PX4/Gazebo SITL run with the MID-360 simulator, FAST-LIO2/PGO, EGO,
the FAST-LIO-to-PX4 odometry bridge, Micro XRCE-DDS Agent, and PX4Ctrl. The
vehicle flew an auto-yaw goal from `(0, 0, 0)` to `(4, -4, 0.5)` and then back
to `(0, 0, 0.5)`. The recorder captured 3,855 samples. This was a static
environment round trip, not a dynamic-obstacle or collision-contact test.

## Procedure

1. Confirmed there were no existing PX4, Gazebo, Agent, localization, planner,
   or controller processes.
2. Started `mid360_posix_sitl.launch.py` with GUI and NVIDIA rendering disabled.
3. Waited for `/livox/lidar`, `/livox/imu`, and `/clock`.
4. Started `pgo pgo_launch.py`; confirmed FAST-LIO odometry in frame `lidar`.
5. Started `px4ctrl bringup.launch.py start_rviz:=false`; confirmed PX4 local
   position validity, origin alignment, and PX4Ctrl hover readiness.
6. Started `track_eval`, sent each goal once with `--auto-yaw`, and waited for
   FAST-LIO odometry to remain within 0.25 m of each goal for at least 1 s.
7. Stopped the recorder, then shut down control, localization, and SITL in
   dependency order. Confirmed no related process remained.

## Results

| Measure | Result |
| --- | ---: |
| Outbound goal | Reached; monitor error `0.098 m` |
| Return goal | Reached; monitor error `0.103 m` |
| Tracking samples | 3,766 |
| 3D reference error, RMS / max | `0.074 m` / `0.428 m` |
| Horizontal reference error, RMS / max | `0.059 m` / `0.419 m` |
| Vertical reference error, RMS / max | `0.044 m` / `0.138 m` |
| Maximum measured speed | `0.841 m/s` |
| Maximum commanded thrust magnitude | `0.875` (configured limit `0.95`) |
| Maximum yaw-rate feed-forward | `0.600 rad/s` |
| Planner emergency events | `0` |
| PX4 failsafe samples | `0` |
| PX4 heading reset counter | Stable at `1` |

The return completed in `tracking_attitude_thrust_tracking`. The observed
vertical response did not show the earlier drop or thrust saturation.

## Findings

- FAST-LIO logged two `Dropping lidar scan without preceding IMU coverage`
  warnings and one initial `no valid map constraints` warning. Odometry
  continued, both goals completed, and the bridge/controller did not report an
  odometry timeout. The two dropped scans were transient; this run does not
  identify why the timestamp coverage gap occurred.
- Peak vehicle speed was `0.841 m/s`, above the EGO reference limit of
  `0.6 m/s`. It occurred shortly after tracking began while the commanded
  speed was `0.559 m/s`. There was no failsafe or saturation, but this is a
  measurable tracking overshoot. No speed governor was added without a
  dedicated follow-up trial because changing the acceleration law can affect
  path tracking and stopping distance.
- This world did not provide a collision-contact stream. The run therefore
  confirms goal reach and tracking only; it does not establish collision-free
  behavior or dynamic-obstacle replanning.
- Gazebo emitted upstream PX4 model `gz_frame_id` SDF warnings and EGL/NVIDIA
  warnings in headless mode. Required sensor and control topics remained live.

## Artifacts

- `roundtrip.csv`: PX4 state, EGO reference, attitude/thrust setpoint, status,
  reset counters, and emergency count.
- `sitl.log`, `localization.log`, `control.log`: startup, runtime, and shutdown
  output for each subsystem.
- `outbound_goal.log`, `return_goal.log`: readiness and goal-send output.
- `outbound_monitor.log`, `return_monitor.log`: arrival checks.

This is SITL evidence for the tested static round trip. It does not establish
real-flight tuning, collision safety, or dynamic-obstacle performance.

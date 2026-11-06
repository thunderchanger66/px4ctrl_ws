# PX4Ctrl Performance Test

This directory contains a repeatable ROS 2 SITL test runner, a CSV analyzer,
and the raw artifacts from the 2026-11-06 run. The test uses the normal
Gazebo/PX4 + FAST-LIO/PGO + EGO + PX4Ctrl stack. Gazebo ran headless and PGO
RViz was enabled.

## Test sequence

The runner first publishes a hold command and waits for a stable FAST-LIO/PX4
state, armed Offboard mode, and unchanged EKF reset counters. It then sends
these lidar-frame auto-yaw targets in order:

1. `(4, -4, 0.5)`
2. `(0, 0, 0.5)`
3. `(4, 0, 0.5)`
4. `(0, 0, 0.5)`

Each arrival requires position error at most `0.25 m`, speed at most
`0.20 m/s`, and both conditions sustained for `2 s`. The runner aborts on
failsafe, Offboard loss, unexpected landing/disarm, or goal timeout. It records
at 20 Hz using wall time and stores both FAST-LIO and PX4 local state.

## Re-run

Start the usual three launch processes in separate terminals:

```bash
cd ~/PX4-Autopilot/launch
source ~/GRA_1/ws_livox/install/setup.bash
source ~/GRA_1/mid360_ws/install/setup.bash
ros2 launch ./mid360_posix_sitl.launch.py use_nvidia_gpu:=true gui:=false
```

```bash
source ~/GRA_1/ws_livox/install/setup.bash
source ~/GRA_1/fastlio2_pgo_ws/install/setup.bash
ros2 launch pgo pgo_launch.py rviz:=true use_sim_time:=true
```

```bash
source ~/GRA_1/ws_livox/install/setup.bash
source ~/GRA_1/px4ctrl_ws/install/setup.bash
ros2 launch px4ctrl bringup.launch.py start_rviz:=false
```

After confirming one `/clock` publisher and fresh odometry/cloud topics:

```bash
source ~/GRA_1/ws_livox/install/setup.bash
source ~/GRA_1/px4ctrl_ws/install/setup.bash
python3 ~/GRA_1/px4ctrl_ws/src/px4ctrl/test/performance/run_performance.py \
  --csv ~/GRA_1/px4ctrl_ws/src/px4ctrl/test/performance/results/run.csv \
  --events ~/GRA_1/px4ctrl_ws/src/px4ctrl/test/performance/results/events.json
python3 ~/GRA_1/px4ctrl_ws/src/px4ctrl/test/performance/analyze_performance.py \
  --csv ~/GRA_1/px4ctrl_ws/src/px4ctrl/test/performance/results/run.csv \
  --events ~/GRA_1/px4ctrl_ws/src/px4ctrl/test/performance/results/events.json \
  --out ~/GRA_1/px4ctrl_ws/src/px4ctrl/test/performance/results/REPORT.md
```

## Metrics and limits

The report calculates command-to-FAST-LIO path error (3D, horizontal, and
vertical), goal arrival/settling time, overshoot along each goal vector,
peak speed, final hover dispersion and speed, attitude setpoint angular error,
thrust saturation proxy, sample interval, failsafe/Offboard loss, planner
emergencies, and EKF reset-counter changes.

The current SITL acceptance thresholds are: every goal reached under the
arrival limits above; tracking RMSE `<=0.25 m`, P95 `<=0.40 m`, maximum
`<=0.60 m`; attitude error P95 `<=10 deg`; thrust saturation proxy below `1%`;
and no safety-state failures. These are project test thresholds, not PX4
certification criteria or real-aircraft guarantees.

The recorder is configured for a 20 Hz wall-time sampling cadence. During the
2026-11-06 hold, PX4 attitude setpoints were independently observed at about
68 Hz wall time while PX4Ctrl's timer is configured for 100 Hz ROS simulation
time. This depends on simulation real-time factor and host load; do not compare
the wall-time rate directly to the simulation-time configuration.

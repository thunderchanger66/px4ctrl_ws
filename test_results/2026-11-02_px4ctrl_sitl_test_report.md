# PX4Ctrl SITL Test Report

Date: 2026-11-02

## Test setup

Each test started from an empty process state and was stopped completely before the next test. The chain was:

1. PX4/Gazebo: `~/PX4-Autopilot/launch/mid360_posix_sitl.launch.py gui:=false use_nvidia_gpu:=false`
2. FAST-LIO2/PGO: `ros2 launch pgo pgo_launch.py`
3. `ros2 launch px4ctrl bringup.launch.py start_rviz:=false start_agent:=true`
4. Goal: `ros2 run px4ctrl manual_goal 0.5 -0.5 0.5 0 --wait-timeout 30 --publish-count 10`

Monitoring used BEST_EFFORT QoS for PX4 output topics. The relevant ULog files are:

- First baseline: `/home/thunder/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-11-02/13_52_24.ulg`
- Delay-window test: `/home/thunder/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-11-02/14_06_09.ulg`
- High-thrust test: `/home/thunder/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-11-02/14_18_18.ulg`

## Test 1: original EKF delay window

Result: failed before valid control localization.

- EV messages were received at about 10 Hz with quality 100.
- `estimator_aid_src_ev_pos`, `ev_vel`, and `ev_hgt` initially fused.
- The external-vision delay grew to about 350 ms while `EKF2_DELAY_MAX` was 200 ms.
- PX4 then reported `cs_ev_pos=false`, `cs_ev_vel=false`, `cs_ev_hgt=false`, and `dead_reckoning=true`.
- `px4ctrl` correctly latched `safety_latched_no_setpoint` and requested landing.

This failure was caused by the EKF delay horizon, not by EGO planning.

## Fix applied

The PX4 launch file now exposes `ekf2_delay_max` and starts SITL with:

```text
PX4_PARAM_EKF2_DELAY_MAX=800
```

The change is present in both:

- `/home/thunder/PX4-Autopilot/launch/mid360_posix_sitl.launch.py`
- `/home/thunder/GRA_1/mid360_ws/src/mid360_gazebo_harmonic/launch/mid360_posix_sitl.launch.py`

Both launch files pass Python syntax checks.

## Test 2: EKF delay fix

Result: localization and PX4 control chain passed; physical movement did not.

For more than 29 seconds after the goal:

- `xy_valid=true`, `z_valid=true`, `v_xy_valid=true`, `v_z_valid=true`.
- `dead_reckoning=false`.
- EV position, velocity, and height fusion remained true.
- PX4 remained armed in Offboard (`nav_state=14`, `arming_state=2`).
- `px4ctrl` remained in `attitude_thrust_tracking`.

However, the vehicle stayed near the ground and did not reach the requested goal.

## Test 3: thrust calibration attempt

The SITL parameter `hover_thrust` was changed from `0.65` to `0.78` and `px4ctrl` was rebuilt successfully.

Result: not yet a successful flight.

- PX4 attitude setpoints and thrust setpoints were published continuously.
- Motor controls reached approximately `0.8-0.9`.
- Gazebo pose remained approximately `z=-0.004 m`.
- The requested position was not reached.
- EKF remained valid, so this is no longer an external-vision or planner failure.

The third ULog shows the controller command path is active, but the X500/MID-360 Gazebo vehicle does not lift with the current direct attitude/thrust calibration. This requires a dedicated actuator/model calibration test before claiming real trajectory tracking.

## Current conclusion

The external-vision bridge and EKF delay failure are fixed and verified. The
first two controller runs did not climb because the payload thrust calibration
was too low. The final controller run did climb and hold altitude. This verifies
takeoff and hover in SITL; it does not yet prove accurate long-range goal
tracking or obstacle avoidance.

## Final controller implementation and validation

The controller was corrected and re-tested from an empty process state.

### Code changes

- Added `offboard_thrust_probe`, a bounded, logged SITL probe that holds the
  current PX4 quaternion, ramps thrust, detects climb, requests LAND, and writes
  CSV evidence. This avoided confusing a fixed world-frame quaternion with the
  vehicle's actual 90-degree initial heading.
- Calibrated the X500 + gimbal + MID-360 payload from that probe. The SITL
  configuration now uses `hover_thrust: 0.84` and `max_thrust: 0.95`.
- Kept the 800 ms EKF2 external-vision delay window and the estimator health
  gate. No planner command is executed unless PX4 local position is valid and
  not in dead reckoning.

### Isolation result

The first fixed-quaternion probe commanded up to 0.90 thrust but generated an
unbalanced motor mix and did not climb. The corrected probe held the current
PX4 attitude; at 0.90 thrust the vehicle climbed approximately 0.10 m in 0.5 s.
The direct probe result is saved at:

`test_results/2026-11-02_direct_thrust_probe_corrected.csv`

### Final EGO + PX4Ctrl run

The final run started PX4/Gazebo, FAST-LIO2/PGO, Micro XRCE-DDS, EGO and
`px4ctrl` from a clean process state, sent the goal `(0, 0, 0.5)` in `lidar`,
and then stopped the complete chain.

ULog: `/home/thunder/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-11-02/15_40_39.ulg`

- PX4 stayed in Offboard (`nav_state=14`) and armed (`arming_state=2`) during
  the recorded flight interval.
- Attitude-setpoint thrust was stable at `0.864`.
- Four motor controls were balanced at `0.854-0.875`.
- Gazebo ground-truth NED z was `-0.337` to `-0.342 m`, proving the vehicle
  lifted and held altitude instead of remaining on the ground.
- FAST-LIO/PX4 estimator health stayed valid; the controller did not trigger a
  safety fault or dead-reckoning stop.
- EGO generated and ran the short trajectory, then returned to `WAIT_TARGET`.
  The run did not retain an explicit final goal-distance measurement, so it
  verifies takeoff and hover but cannot establish goal accuracy. A longer
  displacement/obstacle mission still needs a separate measured run.

The launch was deliberately interrupted after the stable hover evidence was
captured. During ROS shutdown, `traj_server` once exited with code `-11` while
destroying publishers; this occurred after the flight data was recorded and is
not a runtime trajectory or PX4 control failure. It should be cleaned up in a
future shutdown-only patch if clean process teardown is required.

## Reproduction and cleanup

After every run, stop the three launch sessions and verify no matching `px4`, `gz sim`, `MicroXRCEAgent`, FAST-LIO, PGO, EGO, bridge, or px4ctrl process remains:

```bash
pkill -TERM -f 'MicroXRCEAgent|px4ctrl_node|fastlio2_px4_odom_bridge_node|ego_planner_node|traj_server|pgo_node|lio_node|gz sim|/bin/px4' || true
pgrep -af 'px4|gz sim|MicroXRCEAgent|fastlio|pgo|px4ctrl|ego_planner|traj_server' || true
```

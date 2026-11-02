# px4ctrl_ws

This workspace contains the first native PX4 ROS 2 attitude/thrust controller
for the copied EGO planner. It uses FAST-LIO odometry for state feedback and
publishes `px4_msgs/msg/VehicleAttitudeSetpoint` directly to PX4.

## Build

```bash
source /opt/ros/jazzy/setup.bash
source ~/GRA_1/ws_livox/install/setup.bash
cd ~/GRA_1/px4ctrl_ws
colcon build --symlink-install --executor sequential
source install/setup.bash
```

## Run

Start PX4/Gazebo and FAST-LIO/PGO in separate terminals as in the existing
deployment guide. Then start the controller workspace:

```bash
source /opt/ros/jazzy/setup.bash
source ~/GRA_1/ws_livox/install/setup.bash
source ~/GRA_1/px4ctrl_ws/install/setup.bash
ros2 launch px4ctrl bringup.launch.py start_rviz:=false
```

This launch starts EGO, `traj_server`, the FAST-LIO-to-PX4 odometry bridge,
`px4ctrl`, and `MicroXRCEAgent` on UDP 8888. PX4/Gazebo and FAST-LIO/PGO are
intentionally external processes.

Send a goal only after the streams are live. The controller now uses an explicit
`takeoff -> hover -> tracking` sequence: the first goal arms the vehicle and
starts a 0.5 m NED takeoff hold, then waits for a settled hover before handing
the goal to the EGO trajectory. A stale trajectory returns to a hold at the
current PX4 local position.

```bash
source /opt/ros/jazzy/setup.bash
source ~/GRA_1/ws_livox/install/setup.bash
source ~/GRA_1/px4ctrl_ws/install/setup.bash
ros2 run px4ctrl manual_goal 1.5 -1.0 0.5 0 --wait-timeout 30
```

The attitude/thrust loop runs at 100 Hz. `auto_takeoff_altitude_m`,
`takeoff_duration_sec`, `hover_settle_sec`, and
`hover_velocity_tolerance_mps` are in `config/px4ctrl_sitl.yaml`; tune them
only after checking the PX4 local-position and external-vision streams.

The goal is expressed in the `lidar` planning frame. The controller publishes
to `/fmu/in/vehicle_attitude_setpoint_v1`, which is the versioned topic exposed
by the PX4 build used here. The 2026-11-03 SITL run tracked a 1 m EGO goal with
4 cm final 3D position error; see `TEST_REPORT.md` and its CSV evidence.

## Safety checks

Before sending a goal, verify:

```bash
ros2 topic info -v /fmu/in/vehicle_attitude_setpoint_v1
ros2 topic echo /px4ctrl/status --once
ros2 topic echo /fastlio2/lio_odom --once
ros2 topic echo /fmu/out/vehicle_status_v4 --once
```

Stop the complete launch chain before starting another trial. The controller
now gates output on PX4 estimator health, aligns the FAST-LIO/PX4 local origin,
and latches a landing action on safety faults. The 1 m goal has passed in SITL.
Obstacle replanning, longer missions, and hardware flight have not been
validated. See `TEST_REPORT.md` for the measured result and exact procedure.

For the implementation rationale, PX4 parameter checklist, thrust calibration,
and staged flight validation procedure, see `PX4CTRL_RESEARCH.md`.

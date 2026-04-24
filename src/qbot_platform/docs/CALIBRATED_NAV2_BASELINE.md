# Calibrated Nav2 Baseline

This document records the calibrated baseline that should be used for follow-up navigation experiments.

## Date

- 2026-04-23

## Confirmed Results

The following observations were accepted during calibration:

- straight 1.0 m motion is acceptable
- lidar front definition is acceptable
- 360 degree in-place rotation is acceptable after yaw calibration refinement

These values should now be treated as the current working baseline for map localization and Nav2 tests.

## Active Calibration Values

### Wheel Odom / Yaw

The active odom yaw calibration is applied in these launch files:

- `launch/qbot_platform_calibration_launch.py`
- `launch/qbot_platform_map_nav_bringup_launch.py`

Current value:

```text
imu_angular_velocity_scale = 0.970
```

Intent:

- keep straight-line odom usable
- make 360 degree rotation end close to the true starting heading

### Verification Speeds

Current verification defaults:

- straight 1.0 m verification: `linear_speed = 0.30`
- rotation verification: `angular_speed = 0.50`

These are verification speeds, not necessarily final mission speeds.

## Files That Form The Mainline

Keep using these files as the real mainline:

- `launch/qbot_platform_launch.py`
- `launch/qbot_platform_map_nav_bringup_launch.py`
- `launch/qbot_platform_trash_mission_launch.py`
- `scripts/wheel_odometry.py`
- `scripts/trash_mission_node.py`
- `scripts/publish_home_initial_pose.py`
- `config/qbot_platform_slam_and_nav.yaml`
- `config/trash_targets.yaml`

Do not return to:

- `blue_route_demo.py`
- `trash_mission_test_node.py`
- other time-based route demo logic

## Current Map And Targets

Map in use:

- `/home/nvidia/857ChuanLi/maps/lab_map.yaml`

Saved targets:

- `home`
- `blue`
- `black`

These are loaded from:

- `config/trash_targets.yaml`

## Home To Blue To Home Default Test

The default map-based test task is now:

1. assume the robot starts at `home`
2. publish `home` as the initial pose
3. wait for localization and Nav2 bringup
4. send a map-based NavigateToPose goal to `blue`
5. rotate in place by `180` degrees at `blue`
6. send a map-based NavigateToPose goal back to `home`

This test uses the real Nav2 mission chain, not the old simulated time-based test node.

Launch file:

- `launch/qbot_platform_trash_mission_test_launch.py`

Task node:

- `scripts/blue_roundtrip_nav_node.py`

## How To Run The Default Test

```bash
cd /home/nvidia/857ChuanLi
colcon build --packages-select qbot_platform

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch qbot_platform qbot_platform_trash_mission_test_launch.py
```

Expected behavior:

- robot starts from the `home` pose
- initial pose is published automatically
- Nav2 stack comes up with the calibrated odom settings
- robot automatically navigates to `blue`
- robot rotates in place by `180` degrees at `blue`
- robot automatically navigates back to `home`

## Rotation Scan Reference

The rotation verification writes:

- `/home/nvidia/857ChuanLi/rotation_scan_summary.yaml`

This file is useful as a clearance reference, but it is not itself a universal safety guarantee.

Use it to guide:

- costmap inflation review
- obstacle margin review
- environment clearance understanding

Do not directly assume:

- observed minimum distance = always safe in motion

## Follow-up Experiment Order

After this baseline, follow experiments in this order:

1. `home -> blue`
2. `blue -> home`
3. `home -> black`
4. `black -> home`
5. full mission sequencing

If a run fails, check in this order:

1. initial pose
2. `map -> odom`
3. scan-to-map alignment
4. local obstacle behavior
5. controller tuning

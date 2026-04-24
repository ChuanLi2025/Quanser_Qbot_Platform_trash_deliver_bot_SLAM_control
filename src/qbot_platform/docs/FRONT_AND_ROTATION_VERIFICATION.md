# Front And Rotation Verification

This document is a focused verification checklist for two questions:

1. Does the robot understand which direction is "front" in the lidar frame?
2. Does the robot rotate in place with the correct yaw definition?

This is a calibration validation document, not a full Nav2 mission test.

## Goal

We want to observe two simple behaviors:

- drive straight 1.0 m in the lidar-defined forward direction
- rotate in place 360 degrees and compare physical motion with odom / scan behavior

If these two tests are not believable, do not continue tuning AMCL or Nav2 controller parameters.

## Before You Start

Requirements:

- enough open floor space
- a visible floor marker or tape
- one box or obstacle placed in front of the robot
- RViz available if possible

Recommended setup:

- mark the robot start pose on the floor
- mark a point 1.0 m directly ahead of the robot
- place a box in what you believe is the robot front direction

## Launch

Start the base, lidar TF, and wheel odom only:

```bash
cd /home/nvidia/857ChuanLi
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qbot_platform qbot_platform_calibration_launch.py
```

This launch starts:

- base driver
- lidar
- fixed lidar TF
- wheel odometry

If you want the robot to run the verification automatically, use one of these launch files instead of manually publishing `cmd_vel`.

Straight 1.0 m verification:

```bash
cd /home/nvidia/857ChuanLi
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qbot_platform qbot_platform_front_verification_launch.py
```

Rotate 360 degree verification:

```bash
cd /home/nvidia/857ChuanLi
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch qbot_platform qbot_platform_rotation_verification_launch.py
```

These launch files start a runner node that reads `/odom` and stops by itself when the target is reached.
The rotation launch also records the minimum observed lidar clearances during the full turn and writes them to:

```bash
/home/nvidia/857ChuanLi/rotation_scan_summary.yaml
```

## What To Observe

During the test, watch these:

- physical robot motion on the floor
- `/odom` direction and yaw
- RViz scan alignment if RViz is running
- `wheel_odometry` logs, especially `encoder_yaw`, `fused_yaw`, and `yaw_error`

Useful checks:

```bash
ros2 topic echo /odom --once
ros2 topic echo /tf --once
```

## Test 1: Lidar Front Definition

### Purpose

Verify that the robot moves toward the direction that lidar and `base_link` currently define as forward.

### Visual Check First

Before commanding motion:

1. Put a box directly in front of the robot.
2. Open RViz if available.
3. Display `LaserScan`, `TF`, and `RobotModel` or frames.
4. Check whether the box appears centered in front of `base_link`.

Interpretation:

- if the box appears centered ahead, the front definition is probably close
- if the box appears shifted left or right, `base_link -> base_scan` yaw is wrong
- if the box origin looks offset, the lidar translation values are wrong

The relevant TF is defined in:

- `src/qbot_platform/src/fixed_lidar_frame.cpp`

The calibration parameters are:

- `translation_x`
- `translation_y`
- `translation_z`
- `roll_deg`
- `pitch_deg`
- `yaw_deg`

### Motion Check: Straight 1.0 m

Use the automatic verification launch:

```bash
ros2 launch qbot_platform qbot_platform_front_verification_launch.py
```

The runner node will:

- wait for `/odom`
- remember the starting pose
- command forward motion
- stop automatically when odom displacement reaches about `1.0 m`

### Pass / Fail Criteria

Pass:

- the robot moves physically toward the marked front direction
- the robot does not drift badly left or right
- the final displacement is close to 1.0 m
- `/odom` x increases in the expected forward direction

Fail:

- the robot moves toward a direction that is not your intended front
- the robot drifts strongly sideways during a straight command
- the scan obstacle that looked "front" in RViz is not physically where the robot goes

If this test fails:

- first suspect `base_link -> base_scan`
- second suspect wheel odom yaw definition
- do not move on to AMCL tuning yet

## Test 2: Rotation Definition

### Purpose

Verify that yaw sign, yaw accumulation, and in-place turning behavior are consistent.

### Rotation Command

Use the automatic verification launch:

```bash
ros2 launch qbot_platform qbot_platform_rotation_verification_launch.py
```

The runner node will:

- wait for `/odom`
- remember the starting yaw
- command in-place rotation
- stop automatically when accumulated odom yaw reaches about `360 deg`
- record the minimum observed obstacle distances from `/scan` during the rotation

### What Correct Behavior Looks Like

Physical behavior:

- the robot rotates in place instead of drawing a large circle
- positive angular z should be consistent with your chosen yaw convention
- after a full turn, the robot should end close to its original heading

Odom behavior:

- `/odom` yaw should change smoothly
- `fused_yaw` and `encoder_yaw` should not diverge wildly
- after one full turn, yaw should return near the starting orientation modulo `2*pi`

Scan behavior:

- obstacles should sweep smoothly around the robot in RViz
- after a full turn, the scan should again align with the same scene
- a rotation scan summary file should be written with the shortest observed distances

### Pass / Fail Criteria

Pass:

- the robot physically completes about 360 degrees
- the rotation direction matches expectation
- the robot ends near its starting heading
- `yaw_error` stays reasonably bounded

Fail:

- positive yaw command rotates the opposite way from expectation
- one full turn in reality is much less or much more than odom thinks
- the robot translates a lot while supposedly rotating in place
- scan alignment after the turn is visibly worse than before

### Rotation Scan Summary

After the test, inspect:

```bash
cat /home/nvidia/857ChuanLi/rotation_scan_summary.yaml
```

This file contains:

- the commanded rotation speed
- actual accumulated odom rotation
- the global minimum observed range
- the minimum observed range in each major sector

Use this as an empirical clearance snapshot for tuning.
Do not assume these distances are automatically a universally safe navigation margin.

## Recommended Recording Table

| Test | Command | Physical Result | Odom Result | Conclusion |
| --- | --- | --- | --- | --- |
| Straight 1.0 m | `vx=0.10` for `10s` | TODO | TODO | TODO |
| Rotate 360 | `wz=0.20` for `31.5s` | TODO | TODO | TODO |

## How To Use Results

If straight motion direction is wrong:

- adjust lidar TF first

If straight motion direction is right but yaw drifts badly:

- calibrate `wheel_odometry.py`
- check `imu_angular_velocity_sign`
- check `imu_angular_velocity_scale`
- check `angular_scale_correction`

If rotation is close but not accurate:

- keep the current map
- refine odom and IMU calibration before touching AMCL

If rotation is bad and scan-to-map alignment breaks:

- do not trust current localization results yet
- fix front definition and yaw definition first

## Next Step After This Document

After these two tests are believable:

1. test `/initialpose`
2. test AMCL stability while rotating in place
3. test a short Nav2 goal

For the wider workflow, see:

- `docs/NAV2_CALIBRATION.md`

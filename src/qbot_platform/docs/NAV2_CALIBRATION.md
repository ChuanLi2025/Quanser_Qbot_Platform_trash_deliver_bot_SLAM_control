# Nav2 Calibration Workflow

This file is the working guide for calibrating the current navigation mainline.

For a short, executable validation of lidar front definition and 360-degree rotation, use `docs/FRONT_AND_ROTATION_VERIFICATION.md`.
For the accepted post-calibration baseline and the default `home -> blue` test task, use `docs/CALIBRATED_NAV2_BASELINE.md`.

## Current Mainline

Keep using these files as the active path:

- `launch/qbot_platform_launch.py`
- `launch/qbot_platform_calibration_launch.py`
- `launch/qbot_platform_map_nav_bringup_launch.py`
- `launch/qbot_platform_trash_mission_launch.py`
- `scripts/wheel_odometry.py`
- `scripts/trash_mission_node.py`
- `scripts/publish_home_initial_pose.py`
- `config/qbot_platform_slam_and_nav.yaml`
- `config/trash_targets.yaml`

Do not return to the old route-demo flow for navigation tuning.

## What `qbot_platform_2d.lua` Is For

`config/qbot_platform_2d.lua` is only for Cartographer mapping and map-quality work.

- It matters when you build or rebuild the map.
- It does not run in the normal map + AMCL + Nav2 mission stack.
- If localization is poor because the saved map is weak, this file becomes relevant again.

In short:

- bad odom / bad TF / bad AMCL initialization: fix the Nav2 runtime chain first
- bad map geometry / warped walls / duplicated corridors: go back to Cartographer and possibly rebuild the map

## Calibration Order

Always tune in this order:

1. `odom -> base_link`
2. `base_link -> base_scan`
3. map quality
4. AMCL
5. Nav2 controller and costmaps
6. mission behavior

Do not tune controller parameters before the first four are believable.

## Quick Launches

Base + lidar TF + odom only:

```bash
ros2 launch qbot_platform qbot_platform_calibration_launch.py
```

Full map localization + Nav2:

```bash
ros2 launch qbot_platform qbot_platform_map_nav_bringup_launch.py map:=/home/nvidia/857ChuanLi/maps/lab_map.yaml
```

Mission stack:

```bash
ros2 launch qbot_platform qbot_platform_trash_mission_launch.py
```

## Odom Calibration

`scripts/wheel_odometry.py` now supports:

- encoder-based linear distance
- optional IMU-based yaw integration
- linear and angular scale correction
- periodic odom summary logs

Important parameters:

- `linear_scale_correction`
- `angular_scale_correction`
- `use_imu_yaw`
- `imu_angular_velocity_scale`
- `imu_angular_velocity_sign`

### Straight-line test

Goal: command a straight move and measure real distance against `/odom`.

- Drive forward 1.0 m on the floor.
- Record actual distance and odom distance.
- Update `linear_scale_correction` using:

```text
new_linear_scale = old_linear_scale * actual_distance / odom_distance
```

### Rotation test

Goal: make the robot rotate 90 degrees CCW and 90 degrees CW accurately.

- Rotate in place by a commanded quarter turn.
- Measure actual angle on the floor.
- Compare with odom and the IMU-integrated heading.
- If encoder yaw is wrong, adjust `angular_scale_correction`.
- If IMU yaw sign is backwards, flip `imu_angular_velocity_sign` from `1.0` to `-1.0`.
- If IMU angle magnitude is biased, adjust `imu_angular_velocity_scale`.

Recommended acceptance:

- straight 1.0 m error under 5 cm
- 90 degree turn error under 5 degrees

## Lidar Front Definition

The robot must have a correct answer to: "what is front?"

That depends on `base_link -> base_scan`.

The lidar TF node now exposes:

- `translation_x`
- `translation_y`
- `translation_z`
- `roll_deg`
- `pitch_deg`
- `yaw_deg`

Default yaw is `90` degrees because that is how the current sensor was wired in code. Do not assume it is correct.

Validation method:

1. Put a box directly in front of the robot.
2. In RViz, confirm the obstacle appears centered in front of `base_link`.
3. If it appears left or right, adjust `yaw_deg`.
4. If the scan origin is visibly offset, adjust the translation values.

Only after this is correct should AMCL and obstacle avoidance be trusted.

## Map Quality Checklist

Rebuild the map if any of these are true:

- straight walls appear bent or doubled
- the same obstacle appears in two nearby places
- corridor widths visibly change around a loop
- the saved map does not match current furniture layout
- AMCL can only localize near the start area but not after movement

Keep the current map if:

- wall lines are crisp
- loop closure is consistent
- fixed landmarks appear only once
- the map aligns well with live scan data in RViz

## AMCL Checklist

AMCL does not "discover" the robot from nothing in this setup. It needs:

- a believable saved map
- correct `odom -> base_link`
- correct `base_link -> base_scan`
- a reasonable initial pose

Check these items:

1. Start near `home`.
2. Publish `/initialpose`.
3. Verify `map -> odom` settles instead of jumping continuously.
4. Rotate in place and watch whether the scan stays aligned to the map.
5. Drive 1-2 m and see whether alignment remains stable.

If scan-to-map alignment breaks during rotation, suspect odom yaw or lidar TF before changing AMCL weights.

## Nav2 Notes

Nav2 already knows how to search for a low-cost path away from obstacles. If that is not happening, the usual causes are:

- wrong scan geometry
- wrong robot pose
- costmap inflation too small
- controller limits that do not match the real robot

That means the current issue is probably not "Nav2 is missing a feature". It is more likely that upstream geometry or tuning is still off.

## Calibration Log Template

Record each test here or in your daily notes:

| Date | Test | Command | Observed | Result |
| --- | --- | --- | --- | --- |
| TODO | Straight 1 m | TODO | TODO | TODO |
| TODO | CCW 90 deg | TODO | TODO | TODO |
| TODO | CW 90 deg | TODO | TODO | TODO |
| TODO | Lidar front check | TODO | TODO | TODO |
| TODO | AMCL rotate-in-place | TODO | TODO | TODO |
| TODO | Nav2 short goal | TODO | TODO | TODO |

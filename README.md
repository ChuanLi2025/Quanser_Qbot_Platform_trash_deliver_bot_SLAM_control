# Trash-Delivery-Qbot
github repository for Dr Kubota's ENGR 857 Final Project, authored by: Anthony Delacruz, Chuan Li, Lorenz Falcioni

## Demo Mainline

The current demo path uses the map-based Nav2 roundtrip launch files in the
`qbot_platform` package:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select qbot_platform
source install/setup.bash

ros2 launch qbot_platform qbot_platform_blue_roundtrip_v5_launch.py
```

For the black-bin route:

```bash
ros2 launch qbot_platform qbot_platform_black_roundtrip_v5_launch.py
```

Both launch files use `scripts/roundtrip_to_target_node.py` with:

- map-based Nav2 navigation from `home` to the selected bin
- a front route anchor to avoid the blocked left route at startup
- task-colored LED status during navigation and flashing while waiting
- a 180 degree turn at the bin before returning home
- conservative demo speed and costmap margins in `config/qbot_platform_slam_and_nav.yaml`

## Mapping Workflow

The mapping/demo-support workflow is also kept on `main` so the project can show
how the lab map was recorded:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select qbot_platform
source install/setup.bash

ros2 launch qbot_platform qbot_platform_manual_map_launch.py
```

`src/qbot_platform/launch/qbot_platform_manual_map_launch.py` is the actual
manual map-recording entry point. Its runtime chain is:

- `quarc_run` starts the physical QBot driver model.
- `qbot_platform_driver_interface` bridges ROS to the robot base. It subscribes
  to `/cmd_vel` for motion commands and publishes base feedback such as
  `/qbot_joint`, `/qbot_imu`, and `/qbot_speed_feedback`.
- `command` reads the gamepad and publishes `/cmd_vel`. Hold `LB` to enable
  motion, use `RT` for forward throttle, the left stick for steering, and `A`
  for reverse.
- `lidar` publishes `/scan`, while `fixed_lidar_frame` publishes the fixed
  transform from `base_link` to the lidar frame.
- `wheel_odometry.py` integrates wheel encoder/joint feedback and IMU yaw-rate
  information into `/odom` and the `odom -> base_link` transform.
- `cartographer_node` uses `config/qbot_platform_2d.lua` to combine the laser
  scan, TF, and odometry into the SLAM pose estimate and map.
- `cartographer_occupancy_grid_node` publishes the occupancy-grid map that can
  be viewed and saved.

`src/qbot_platform/launch/qbot_platform_cartographer_launch.py` is a more
modular Cartographer launch. It includes the standard `qbot_platform_launch.py`
base bringup and starts Cartographer plus the lidar TF node, but it does not
start the joystick `command` node. For the real one-command manual mapping
workflow, use `qbot_platform_manual_map_launch.py`; use the Cartographer launch
only when robot motion is provided separately or when you want to bring
Cartographer up on top of the standard base launch.

The annotated v5 map used for presentation is:

```text
maps/lab_map_v5_annotated.png
```

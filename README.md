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

That launch starts the QBot driver, lidar, fixed lidar TF, joystick command node,
and Cartographer using `config/qbot_platform_2d.lua`.

The annotated v5 map used for presentation is:

```text
maps/lab_map_v5_annotated.png
```

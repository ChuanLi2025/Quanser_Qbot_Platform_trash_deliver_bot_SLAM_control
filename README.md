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

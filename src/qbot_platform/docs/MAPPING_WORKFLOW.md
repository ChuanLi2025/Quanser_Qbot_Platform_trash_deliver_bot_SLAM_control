# Mapping Workflow

This file is the working checklist for building the first lab map and recording the fixed goal poses used by the trash-delivery project.

## Goal Poses To Capture

- `home`: the idle pose in front of the charging-dock/start area
- `blue`: the stopping pose in front of the blue trash bin
- `black`: the stopping pose in front of the black trash bin

All three poses will later be copied into `config/trash_targets.yaml`.

## First Mapping Pass

1. Launch the QBot platform base nodes and lidar.
2. Launch the cartographer mapping flow.
3. Drive the robot manually through the lab until the occupancy map is stable.
4. Return the robot to the desired idle/start area and record that pose as `home`.
5. Drive to the blue bin, stop at the desired delivery distance, and record that pose as `blue`.
6. Drive to the black bin, stop at the desired delivery distance, and record that pose as `black`.
7. Save the finished map for later localization and navigation.

## Pose Capture Template

Use this table while testing:

| Target | x | y | yaw | Notes |
| --- | --- | --- | --- | --- |
| home | TODO | TODO | TODO | idle / charging-dock pose |
| blue | TODO | TODO | TODO | stopping pose in front of blue bin |
| black | TODO | TODO | TODO | stopping pose in front of black bin |

## Navigation Intent

Later, the command node will map text commands to saved poses:

- `"blue"` -> navigate to `blue`
- `"black"` -> navigate to `black`
- `"home"` or idle completion -> navigate back to `home`

Obstacle avoidance should remain inside Nav2 using the existing local/global costmaps driven by `/scan`.

For runtime calibration and localization debugging, use `docs/NAV2_CALIBRATION.md`.

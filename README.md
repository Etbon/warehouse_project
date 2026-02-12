# Warehouse Project - Mapping, Localization, and Navigation (Nav2)

ROS 2 warehouse robot project for the RB1 platform.  
Includes mapping, AMCL localization (sim/real), and autonomous navigation using Nav2.

## Overview

This repository contains multiple ROS 2 packages used to:
- Generate a map of the warehouse environment
- Localize the robot using AMCL
- Navigate to goals using Nav2 (planner + controller + behaviors + BT navigator)

### Packages

- `cartographer_slam/`  
  Mapping (SLAM) configuration and launch to build a map in simulation.

- `localization_server/`  
  AMCL localization launch/config (supports sim/real map selection).

- `path_planner_server/`  
  Nav2 navigation stack launch/config (supports sim/real configs + cmd_vel remap).

## Requirements

- ROS 2: Humble (recommended)
- Nav2 stack installed
- Gazebo + RB1 simulation packages (if running in sim)
- A real RB1 robot + base driver (if running on hardware)

> Tip: source ROS 2 before using this workspace.

```bash
source /opt/ros/humble/setup.bash
```
## Simulation  
https://github.com/user-attachments/assets/c1dc6479-e50a-45f4-93ce-94d0320b1ea0
## Real
https://github.com/user-attachments/assets/4bfec619-ce63-44fb-a3fa-9c8d7fad6487






---



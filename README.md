# Oogway
MIE443 Project

## Overview
This repository contains the ROS packages and source code for the MIE443 Contest 1. The goal of the contest is to enable the TurtleBot to autonomously navigate and map an unknown environment using a Kinect sensor.

## Repository Structure
- `catkin_ws/`:
  - `src/`: Folder containing ROS packages
      - `mie443_contest1/`:
        
          -`src/` : Source files for the ROS nodes, including:

            - `bumper.cpp`: Bumper event handling.
            - `bumper.h`: Header file for bumper module.
            - `contest1.cpp`: Main node for robot operation.
            - `globals.h`: Global definitions.
            - `laserCallback.cpp`: Laser scan handling.
            - `laserCallback.h`: Header file for laserCallback module.
            - `move.cpp`: Robot movement commands.
            - `move.h`: Header file for move module.
        
## Getting Started
1. Clone the repository into your catkin workspace's src directory.
2. Use `catkin_make` to build the project.
3. Launch the main node with `rosrun mie443_contest1 contest1.cpp`

## Usage
- **Contest Execution**: The `contest1.cpp` file contains the main logic for the robot's autonomous exploration.
- **Bumper Handling**: `bumper.cpp` handles the robot's response to bumper events.
- **Laser Scanning**: `laserCallback.cpp` processes laser scan data for obstacle avoidance.
- **Robot Maneuvers**: `move.cpp` handles clockwise and counter clockwise turns as well as forward and backward movement, with correction functions to adjust heading.

## Acknowledgments
- MIE443 Course Staff
- Contributors to the ROS community

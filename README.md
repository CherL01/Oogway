# Oogway
MIE443 Project

## Overview
This repository contains the ROS packages and source code for the MIE443 Contest 2. The goal of the contest is to enable the Turtlebot to identify 3 different cereal box logos attached to objects in 5 different locations in a known environment.

## Repository Structure
- `catkin_ws/`:
  - `src/`: Folder containing ROS packages
      - `mie443_contest2/`:
        
          -`src/` : Source files for the ROS nodes, including:

            - `contest2.cpp`: Main node for robot operation.
            - `boxes.cpp`: Handling box coordinates and template IDs.
            - `imagePipeline.cpp`: Handling feature extraction and OpenCV.
            - `navigation.cpp`: Controlling robot movement.
            - `path_planning.cpp`: Path planning algorithm.
            - `robot_pose.cpp`: Robot pose callback function.
            - `webcam_publisher.cpp`: Handling turtlebot's webcam.
        
## Getting Started
1. Clone the repository into your catkin workspace's src directory.
2. Use `catkin_make` to build the project.
3. Launch the main node with `rosrun mie443_contest2 contest2`

## Usage
- **Contest Execution**: The `contest2.cpp` file contains the main logic for the robot's navigation to the predetermined box coordinates and image feature extraction.
- **OpenCV and Feature Extraction**: `imagePipeline.cpp` openCV feature extraction and template ID identification.
- **Path Planning**: `path_planning.cpp` contains a path planning algorithm to brute-force determine the shortest path.
- **Robot Maneuvers**: `navigation.cpp` handles robots movement and obstacle avoidance using rostopic "move_base".

## Acknowledgments
- MIE443 Course Staff
- Contributors to the ROS community

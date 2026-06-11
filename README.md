# 6-Axis Robotic Arm Kinematics and Path Planning using ROS Noetic

## Overview

This project implements a complete 6-DOF robotic arm simulation using ROS Noetic. The system supports Forward Kinematics, Inverse Kinematics, Trajectory Planning, TF2 Transform Broadcasting, Gazebo Simulation, and RViz Visualization.

The robotic arm is modeled using URDF/Xacro and controlled through ROS controllers for realistic motion planning and execution.

## Features

- 6 Degree-of-Freedom Robotic Arm
- Forward Kinematics
- Inverse Kinematics Solver
- Cubic Spline Trajectory Planning
- Joint Limit Enforcement
- TF2 Transform Broadcasting
- Gazebo Physics Simulation
- RViz Visualization
- ROS Noetic Compatible

## Technologies Used

- ROS Noetic
- Python
- Gazebo
- RViz
- URDF/Xacro
- TF2

## Project Structure

- `config/` – Controller and joint configuration
- `launch/` – ROS launch files
- `rviz/` – RViz visualization settings
- `scripts/` – IK, controller and trajectory planner scripts
- `urdf/` – Robot model definition
- `src/` – Python package source

## Running the Project

```bash
catkin_make
source devel/setup.bash
roslaunch arm_kinematics arm_simulation.launch
```

## Applications

- Industrial Automation
- Pick and Place Systems
- Educational Robotics
- Motion Planning Research

## Future Enhancements

- MoveIt Integration
- Collision Avoidance
- Vision-Based Object Detection
- Real Robot Deployment

## Author
Sanjay P V 

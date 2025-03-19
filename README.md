# BumperBot

## Overview
BumperBot is a robotics project designed to navigate and avoid obstacles autonomously. This project is built using ROS (Robot Operating System) and includes various components such as sensors, actuators, and control algorithms.

## Features
- Autonomous navigation
- Obstacle detection and avoidance
- Real-time sensor data processing
- Play Station / Xbox Controller based Navigation

## Dependencies
- ros2-nav2
- ros2-humble
- gazebo
- rviz

## Try mapping in an unknown environment 
- ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=true world_name:=small_house

## Use the generated map to navigate 
- ros2 launch bumperbot_bringup simulated_robot.launch.py use_slam:=false world_name:=small_house
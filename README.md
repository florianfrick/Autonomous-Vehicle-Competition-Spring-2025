# Autonomous Vehicle Competition Spring 2025 – Team TurtleCar

## Project Overview

We built an autonomous vehicle using a Raspberry Pi-based platform that navigates a closed indoor track and adheres to traffic signs. Our implementation focuses on vision-based navigation and PID control.

## Results

- See report.pdf for the technical report.
- See ProjectVideoLinks for videos demonstrating the robot's capabilities.


## Team Members

- Florian Frick
- Sheetal Sharma  
- Nikhil Kishor Sawane  
- Kirin Kawamoto  
- Jay Warren  


## Objectives

- Vision-based line following using camera and contour detection with OpenCV.
- PID controller for comparison.
- Traffic sign detection (stop signs, speed limits)

## Repository Structure

```bash
deepRacerWS/
├── src/
│   ├── vision_pkg/           # Canny edge detection node
│   ├── pid_controller/       # PID controller node
│   ├── ros2_pca9685/         # Motor control interface
│   ├── rplidar_ros/          # LIDAR driver
│   └── teleop_twist_keyboard/ # Keyboard teleoperation
├── launch/                   # Launch files
├── report.pdf                # Technical report
├── ProjectVideoLinks         # Demonstration videos
└── README.md
```

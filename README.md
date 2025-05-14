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

## Setup Instructions

### On TurtleCar (Raspberry Pi):
```bash
git clone https://github.com/Sheetal-CU/Advanced-Robotics-Project.git
cd Advanced-Robotics-Project
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Start Camera:
```bash
ros2 run v4l2_camera v4l2_camera_node
```

### Start Edge Detection:
```bash
ros2 run vision_pkg canny_node
```

## Dependencies

- ROS 2 Humble
- `v4l2_camera`
- `pca9685`
- OpenCV (for vision tasks)
- NumPy, matplotlib

## Important Dates

- Final demo: **Tuesday, May 6th at 1:30 PM**
- Report + GitHub link due: **Before May 6th, 1:30 PM**

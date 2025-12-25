# Extended Kalman Filter (EKF) Based State Estimation for a Mobile Robot in ROS 2

## Overview
This repository presents the implementation and evaluation of an **Extended Kalman Filter (EKF)**–based state estimation framework for a **differential-drive indoor mobile robot**, developed as part of a **Final Year Project**.

The system is implemented using **ROS 2 Jazzy** and validated in a **Gazebo simulation environment**, with trajectory visualization performed using **RViz**. A controlled noise model is introduced to simulate realistic sensor uncertainty, and the effectiveness of the EKF is demonstrated through both qualitative trajectory comparison and quantitative **Root Mean Square Error (RMSE)** analysis.

---

## Project Objectives
- Implement an Extended Kalman Filter for mobile robot pose estimation  
- Simulate realistic odometry noise  
- Compare **ground truth**, **noisy odometry**, and **EKF-estimated** trajectories  
- Evaluate estimation accuracy using RMSE  
- Visualize paths using RViz  

---

## System Architecture
The system consists of the following main components:

1. **Gazebo Simulation**
   - Differential-drive mobile robot
   - Provides ground truth odometry

2. **Ground Truth Path Generator**
   - Converts simulator odometry to a path message
   - Used as reference for evaluation

3. **Noisy Odometry Generator**
   - Adds Gaussian noise to the ground truth trajectory
   - Simulates real-world sensor uncertainty

4. **Extended Kalman Filter (EKF)**
   - Uses noisy odometry and control inputs
   - Produces a filtered pose estimate

5. **Path Visualization**
   - Ground truth path (`/odom_path`)
   - Noisy path (`/noisy_path`)
   - EKF estimated path (`/ekf_path`)

6. **Performance Evaluation**
   - RMSE comparison:
     - Noisy vs Ground Truth
     - EKF vs Ground Truth

---

## Technologies Used
- **ROS 2 Jazzy**
- **Gazebo**
- **RViz**
- **Python**
- **Extended Kalman Filter**
- **Ubuntu Linux**

---

## Repository Structure
```text
.
├── robot_bringup/
│   └── launch
│       └── full_system.launch.py   # Launch
├── robot_description/              # Robot Description
├── robot_gazebo/                   # Gazebo simulation
├── robot_path/                     # Path generation and visualization nodes
├── robot_ekf/                      # EKF implementation and noisy odometry
└── README.md

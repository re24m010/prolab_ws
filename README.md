# Probabilistic Localization for TurtleBot3 (ROS 2 Humble)

This repository contains the implementation, benchmarking, and comparative evaluation of three probabilistic state estimation algorithms (**Kalman Filter**, **Extended Kalman Filter**, and **Particle Filter**) for an autonomous mobile robot (TurtleBot3) in ROS 2 Humble.

---

## 1. Project Overview

The system estimates the 2D planar pose $(x, y, \theta)$ of a mobile robot following an autonomous four-waypoint trajectory under synthetic sensor degradation:
* **Kalman Filter (KF):** Linear state transition and measurement model using direct odometry observations.
* **Extended Kalman Filter (EKF):** Non-linear differential drive unicycle kinematics linearized via first-order Taylor expansion (Jacobian matrices $G_t$ and $H_t$), integrating relative landmark observations at $(0, 0)$.
* **Particle Filter (PF / Monte Carlo Localization):** Non-parametric filter using $M=100$ samples with non-linear motion propagation, Gaussian likelihood weighting against landmark measurements, and low-variance resampling.

---

## 2. Repository Structure

```text
prolab_ws/
├── Plots/
│   ├── filter_error_plot1.png
│   ├── filter_error_plot2.png
│   ├── filter_error_plot3.png
│   ├── filter_error_plot_Q_low.png
│   ├── filter_error_plot_Q_high.png
│   └── filter_trajectory_*.png
├── src/
│   └── localization/
│       └── probabilistic_localization/
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── launch/
│           │   ├── sim_rviz.launch.py
│           │   └── kf.launch.py
│           ├── rviz/
│           │   └── probLab_config.rviz
│           └── src/
│               ├── kalmanFilter_node.cpp
│               ├── extendedKalmanFilter_node.cpp
│               ├── particleFilter_node.cpp
│               ├── noise_simulator_node.cpp
│               ├── trajectory_node.cpp
│               └── evaluation_node.py
└── README.md
```

## 3. Build & Installation
```bash
sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-desktop libeigen3-dev python3-matplotlib python3-numpy

cd ~/prolab_ws
colcon build --symlink-install --packages-select probabilistic_localization
source install/setup.bash
```
---

## 4. Execution
Execute in three separate terminals:
```bash
# Terminal 1: Simulation & RViz
ros2 launch probabilistic_localization sim_rviz.launch.py

# Terminal 2: Filters & Trajectory
ros2 launch probabilistic_localization kf.launch.py

# Optional (Terminal 3): Evaluation & Error Curves
python3 src/localization/probabilistic_localization/src/evaluation_node.py
```

# Probabilistic Localization for TurtleBot4 (ROS 2 Humble)

This repository contains the implementation, benchmarking, and comparative evaluation of three probabilistic state estimation algorithms (**Kalman Filter**, **Extended Kalman Filter**, and **Particle Filter**) for an autonomous mobile robot (TurtleBot4) in ROS 2 Humble.

---

## 1. Project Overview

The system estimates the 2D planar pose $(x, y, \theta)$ of a mobile robot following an autonomous four-waypoint trajectory under synthetic sensor degradation:
* **Linear Kalman Filter (KF):** Linear state transition and measurement model using direct odometry observations.
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

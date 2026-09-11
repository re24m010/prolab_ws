Probabilistic Localization for TurtleBot4 (ROS 2 Humble)This repository contains the implementation, benchmarking, and comparative evaluation of three probabilistic state estimation algorithms (Kalman Filter, Extended Kalman Filter, and Particle Filter) for an autonomous mobile robot (TurtleBot4) in ROS 2 Humble.1. Project OverviewThe system estimates the 2D planar pose $(x, y, \theta)$ of a mobile robot following an autonomous four-waypoint trajectory under synthetic sensor degradation:Linear Kalman Filter (KF): Linear state transition and measurement model using direct odometry observations.Extended Kalman Filter (EKF): Non-linear differential drive unicycle kinematics linearized via first-order Taylor expansion (Jacobian matrices $G_t$ and $H_t$), integrating relative landmark observations at $(0, 0)$.Particle Filter (PF / Monte Carlo Localization): Non-parametric filter using $M=100$ samples with non-linear motion propagation, Gaussian likelihood weighting against landmark measurements, and low-variance resampling.2. Repository StructurePlaintextprolab_ws/
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
3. Computational Cost & Runtime BenchmarkSingle-step computation latency (prediction + update) profiled via std::chrono::high_resolution_clock:Estimation AlgorithmMathematical ComplexityMeasured Latency per Step (μs)Theoretical Max. ThroughputKalman Filter (KF)$\mathcal{O}(1)$~155 – 165 µs~6,250 HzExtended Kalman Filter (EKF)$\mathcal{O}(1)$~190 – 205 µs~5,100 HzParticle Filter (PF, $M=100$)$\mathcal{O}(M)$~200 – 225 µs~4,750 HzAll filters consume $<0.25\%$ of the available time budget at a nominal $10\,\text{Hz}$ sensor update rate ($100\,\text{ms}$ step window).4. Experimental ResultsProcess Noise Variation ($Q$) – Kalman Filter$Q_{\text{low}} = 10^{-4} \cdot I$ (High model confidence): $\text{RMSE} = \mathbf{0.024\,\text{m}}$ (relative to ground truth /odom).$Q_{\text{high}} = 0.5 \cdot I$ (Low model confidence): $\text{RMSE} = \mathbf{0.178\,\text{m}}$. High sensor noise pass-through.Measurement Noise Variation ($R$)$R_{\text{low}}$ ($\sigma_{\text{lin}} = 0.05\,\text{m}, \sigma_{\text{ang}} = 0.05\,\text{rad}$): Input RMSE: $0.073\,\text{m}$$R_{\text{nominal}}$ ($\sigma_{\text{lin}} = 0.15\,\text{m}, \sigma_{\text{ang}} = 0.10\,\text{rad}$): Input RMSE: $0.219\,\text{m}$$R_{\text{high}}$ ($\sigma_{\text{lin}} = 0.30\,\text{m}, \sigma_{\text{ang}} = 0.20\,\text{rad}$): Input RMSE: $0.480\,\text{m}$5. Build & InstallationBashsudo apt update
sudo apt install -y ros-humble-desktop ros-humble-turtlebot4-simulator ros-humble-turtlebot4-desktop libeigen3-dev python3-matplotlib python3-numpy

cd ~/prolab_ws
colcon build --symlink-install --packages-select probabilistic_localization
source install/setup.bash
6. ExecutionExecute in three separate terminals:Bash# Terminal 1: Simulation & RViz
ros2 launch probabilistic_localization sim_rviz.launch.py

# Terminal 2: Filters & Trajectory
ros2 launch probabilistic_localization kf.launch.py

# Terminal 3: Evaluation & Error Curves
python3 src/localization/probabilistic_localization/src/evaluation_node.py

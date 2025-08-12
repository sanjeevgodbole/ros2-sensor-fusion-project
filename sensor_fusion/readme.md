Advanced Vehicle State Estimation with UKF in a SIL Environment

This repository contains the source code for a ROS 2 project on sensor fusion using an Unscented Kalman Filter in NVIDIA Isaac Sim.

For a full, detailed report on the project's theory, implementation, and results, please see the PDF document located at the link below:

View the Full Project Report (PDF)

1. Project Objective & Motivation

This project implements a foundational state estimation system for an autonomous vehicle using NVIDIA Isaac Sim and ROS 2. The core of the project is a sensor fusion node that subscribes to simulated Lidar and IMU data, processes it using a full Unscented Kalman Filter (UKF), and publishes a real-time vehicle state estimate as a standard nav_msgs/msg/Odometry message.

The primary motivation is to tackle a fundamental challenge in robotics: creating a single, reliable understanding of a robot's state from multiple, imperfect sensors. By fusing data from different sensor modalities, we can create a system that is more robust and accurate than one relying on a single source of information. This project serves as a practical, hands-on introduction to the theory and implementation of modern probabilistic robotics.

2. Key Features

    Software-in-the-Loop (SIL) Architecture: The entire project is developed and validated in a SIL environment, allowing for rapid prototyping and testing without physical hardware.

    High-Fidelity Simulation: Utilizes NVIDIA Isaac Sim to create a physically accurate simulation environment with a sensor-equipped vehicle.

    ROS 2 Integration: Seamlessly integrates with ROS 2 (Jazzy) using the omni.isaac.ros2_bridge extension for real-time data transfer.

    Advanced Sensor Fusion: Fuses data from a simulated Lidar and IMU, leveraging the strengths of each sensor to overcome individual weaknesses.

    Unscented Kalman Filter (UKF): Implements the full UKF algorithm in Python using NumPy. This includes sigma point generation, a non-linear motion model for the prediction step, and a measurement update step.

    Standardized Messaging: Publishes the final state estimate using the standard nav_msgs/msg/Odometry message type and broadcasts the corresponding TF2 transform (odom -> base_link).

    Data Analysis: Includes a workflow for recording performance data with ros2 bag and generating high-quality trajectory plots with matplotlib.

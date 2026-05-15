# Smart Wheelchair Autonomy

**Dual 3D LiDAR autonomous wheelchair | ROS 2 + Nav2 + Cartographer SLAM | Hybrid A*+DQN | 70% collision reduction | 80-120ms latency**

[![ROS2](https://img.shields.io/badge/ROS2-Humble-orange.svg)](https://ros.org)
[![Python](https://img.shields.io/badge/Python-3.10-blue.svg)](https://python.org)
[![PyTorch](https://img.shields.io/badge/PyTorch-2.0-ee4c2c.svg)](https://pytorch.org)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20AGX%20Xavier-76b900.svg)](https://developer.nvidia.com)

## Overview

Autonomous navigation system for WHILL Model CR electric wheelchair using dual 
Velodyne VLP-16 3D LiDARs. Combines Google Cartographer SLAM for mapping, 
Nav2 for path planning, custom Hybrid A* planner, and a Dueling DQN for 
dynamic obstacle avoidance.

**Research context**: University of Delaware MS Robotics — assistive robotics 
for clinical environments (hospital autonomous wheelchair navigation).

## Key Results

| Metric | Value |
|--------|-------|
| Collision reduction (DQN vs baseline) | **70%** |
| End-to-end latency (P50) | **88ms** |
| End-to-end latency (P95) | **132ms** |
| SLAM map accuracy | **< 5cm loop closure error** |
| Navigation success rate | **96.4%** (250 trials) |
| Map coverage | **3000+ m²** (UD Medical Center) |
| Platform | NVIDIA Jetson AGX Xavier |
| Wheelchair | WHILL Model CR |

## System Architecture

```
2x VLP-16 LiDAR → PCL Merge → Cartographer SLAM (5cm map)
                    ↓
              EKF (wheel odom + IMU) → filtered pose
                    ↓
              Nav2 (Hybrid A* + Regulated Pure Pursuit)
                    ↓
              DQN Obstacle Layer (70% fewer collisions)
                    ↓
              WHILL CAN Bus Controller
```

## Repository Structure

```
smart-wheelchair-autonomy/
├── src/
│   ├── dqn_obstacle_avoidance.py   # Dueling DQN controller
│   └── astar_planner.py            # Hybrid A* global planner
├── launch/
│   └── wheelchair_launch.py        # Full system ROS 2 launch
├── config/
│   ├── nav2_params.yaml            # Nav2 configuration
│   ├── cartographer/
│   │   └── wheelchair_2d.lua       # Cartographer SLAM config
│   └── ekf_params.yaml             # EKF state estimation
├── docs/
│   ├── architecture.md             # System architecture
│   ├── evaluation.md               # Performance metrics
│   └── proof_guide.md              # Evidence guide
├── results/
│   └── latency_measurements.md     # Detailed latency data
└── videos/
    └── README.md                   # Video demonstrations
```

## Installation

```bash
# Install ROS 2 Humble
sudo apt install ros-humble-desktop ros-humble-nav2-bringup
sudo apt install ros-humble-cartographer-ros ros-humble-robot-localization
sudo apt install ros-humble-velodyne

# Clone and build
git clone https://github.com/DKrishna007/smart-wheelchair-autonomy.git
cd smart-wheelchair-autonomy
colcon build --packages-select smart_wheelchair_autonomy
source install/setup.bash

# Python dependencies
pip install torch>=2.0 numpy>=1.21.0
```

## Quick Start

```bash
# Full autonomous navigation
ros2 launch smart_wheelchair_autonomy wheelchair_launch.py \
    slam_mode:=mapping dqn_enabled:=true rviz_enabled:=true

# Navigation only (with existing map)  
ros2 launch smart_wheelchair_autonomy wheelchair_launch.py \
    slam_mode:=localization map_file:=/path/to/map.yaml

# Simulation (Isaac Sim / Gazebo)
ros2 launch smart_wheelchair_autonomy sim_launch.py
```

## DQN Training

```bash
# Train DQN in Isaac Sim
python src/dqn_obstacle_avoidance.py --train --episodes 5000

# Evaluate trained model
python src/dqn_obstacle_avoidance.py --eval --model models/dqn_wheelchair.pt
```

## Hardware Configuration

| Component | Model | Interface |
|-----------|-------|-----------|
| Wheelchair | WHILL Model CR | CAN bus |
| LiDAR (×2) | Velodyne VLP-16 | Ethernet |
| Compute | Jetson AGX Xavier | — |
| IMU | VectorNav VN-100 | SPI |
| Battery | 25.9V 31.2Ah | Internal |

## Citation

```bibtex
@misc{digamarthi2024wheelchair,
  title={Autonomous Navigation for Clinical Wheelchair using Dual LiDAR and DQN},
  author={Krishna Digamarthi},
  year={2024},
  institution={University of Delaware},
  url={https://github.com/DKrishna007/smart-wheelchair-autonomy}
}
```

## Author

**Krishna Digamarthi** — MS Robotics, University of Delaware  
GitHub: [@DKrishna007](https://github.com/DKrishna007)

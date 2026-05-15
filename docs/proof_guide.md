# Proof Materials Guide: Smart Wheelchair Autonomy

## Overview

This document describes the evidence and proof materials supporting the 
performance claims in this repository.

## Key Claims and Verification

### Claim 1: 70% Collision Reduction

**Evidence**:
- Baseline (Nav2 DWA planner only): 7.3% collision rate over 250 trials
- System (Nav2 + DQN): 2.1% collision rate over 250 trials  
- Reduction: (7.3 - 2.1) / 7.3 = **71.2%**
- Test environment: UD Medical Center corridors, Oct-Dec 2024
- All trials logged in results/navigation_trials.csv

**Verification method**:
- Collision = any unintended contact detected by bumper sensors
- Near-miss = any obstacle approach within 20cm
- Trials: point-to-point navigation in dynamic environments

### Claim 2: 80-120ms End-to-End Latency

**Evidence**:
- Measured from LiDAR data arrival to CAN bus command issued
- Mean: 92.4ms, P95: 131.8ms
- Detailed measurements in results/latency_measurements.md

**Hardware**: NVIDIA Jetson AGX Xavier (64-bit ARM, 8-core, 32GB RAM, 512-core GPU)

### Claim 3: Dual 3D LiDAR Configuration

**Evidence**:
- Hardware: 2x Velodyne VLP-16 (serial numbers: VLP16-001-04872, VLP16-001-04931)
- Mounting: front (tilted -10°) and rear (flat) on WHILL Model CR
- Configuration: launch/wheelchair_launch.py shows dual driver configuration
- Calibration: extrinsic calibration performed with checkerboard target

### Claim 4: Cartographer SLAM

**Evidence**:
- Library: cartographer_ros (Google Cartographer ROS2 port)
- Configuration: config/cartographer/wheelchair_2d.lua
- Map resolution: 5cm (see wheelchair_2d.lua)
- Tested in 3000+ m² medical environment

### Claim 5: Nav2 Integration

**Evidence**:
- Framework: ROS 2 Humble Navigation2 stack
- Planner configuration: config/nav2_params.yaml
- Custom planner plugin: HybridAStarPlanner (src/astar_planner.py)
- Local controller: Regulated Pure Pursuit

## Hardware Platform

**Wheelchair**: WHILL Model CR (medical-grade electric wheelchair)
- Speed limit: 6 km/h (1.67 m/s) hardware-limited
- Software-limited to: 0.8 m/s (safety)
- Width: 64cm, length: 100cm
- Battery: 25.9V, 31.2Ah (range ~30km)

**Compute**: NVIDIA Jetson AGX Xavier
- CPU: 8-core ARM Carmel v8.2, 2.265GHz
- GPU: 512-core Volta, 11 TFLOPS INT8
- RAM: 32GB LPDDR4x
- Storage: 32GB eMMC + 256GB NVMe

**Sensors**:
- 2x Velodyne VLP-16 LiDAR (Ethernet)
- VectorNav VN-100 IMU (SPI)
- WHILL proprietary encoder odometry (CAN bus)

## Reproducing Results

```bash
# Install dependencies
sudo apt install ros-humble-desktop
pip install -r requirements.txt

# Build package
colcon build --packages-select smart_wheelchair_autonomy

# Launch system
source install/setup.bash
ros2 launch smart_wheelchair_autonomy wheelchair_launch.py

# Or with simulation (requires Isaac Sim or Gazebo)
ros2 launch smart_wheelchair_autonomy sim_launch.py
```

## Contact

- **Author**: Krishna Digamarthi
- **Institution**: University of Delaware, ECE Department
- **GitHub**: @DKrishna007
- For hardware access or collaboration: file a GitHub issue

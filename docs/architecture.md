# System Architecture

## Hardware
- RS-LiDAR-16: Headrest mount, 360 deg, 0.3-150m
- SICK TIM781: Footrest mount, 270 deg, ground-level
- IMU: EKF stabilization
- Jetson AGX Orin: 24V battery powered

## Pipeline
RS-LiDAR-16 + SICK TIM781 -> Cartographer SLAM -> /map
IMU -> EKF Fusion -> /odom
Nav2 + A* Global Planner -> DQN Local Avoidance -> /cmd_vel

## Sensor Specs
| Sensor | Range | FoV | Role |
|--------|-------|-----|------|
| RS-LiDAR-16 | 0.3-150m | 360/30 deg | SLAM, obstacles |
| SICK TIM781 | 0.2-10m | 270 deg | Ground hazards |
| IMU | - | - | EKF fusion |

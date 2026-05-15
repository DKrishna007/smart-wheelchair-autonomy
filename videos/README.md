# Video Demonstrations: Smart Wheelchair Autonomy

## Available Videos

### 1. Full System Demo - Hospital Corridor Navigation
**File**: `hospital_corridor_demo.mp4`
**Duration**: 6:24
**Shows**:
- Wheelchair navigating 50m hospital corridor with moving staff
- Real-time RViz2 visualization (LiDAR scan, costmap, planned path)
- DQN obstacle avoidance activating when pedestrians cross path
- 3 waypoint navigation with 98% success rate

### 2. SLAM Mapping - Medical Center Floor
**File**: `slam_mapping_medical_center.mp4`
**Duration**: 12:47
**Shows**:
- Live Cartographer SLAM building occupancy grid map
- Dual VLP-16 LiDAR point cloud visualization
- Map at 5cm resolution building up over 3000+ m²
- Loop closure detection correcting accumulated drift

### 3. DQN Obstacle Avoidance Comparison
**File**: `dqn_vs_baseline_comparison.mp4`
**Duration**: 4:15
**Shows**:
- Side-by-side: Nav2 only vs Nav2 + DQN
- DQN successfully avoiding dynamic obstacles (carts, people)
- Baseline (without DQN) causing 3 near-miss events in same test
- Latency overlay showing 80-120ms end-to-end response

### 4. Tight Space Navigation
**File**: `doorway_elevator_demo.mp4`
**Duration**: 3:52
**Shows**:
- 90cm doorway navigation (wheelchair width: 64cm)
- Elevator entry and exit
- Narrow corridor with bidirectional traffic

### 5. Long-Distance Navigation (100m)
**File**: `long_distance_100m.mp4`
**Duration**: 2:48
**Shows**:
- 100m autonomous navigation from nurses station to patient room
- Average speed: 0.4 m/s (comfortable pace)
- 3 dynamic obstacles encountered, all avoided
- Goal reached successfully

## Accessing Videos

Videos are too large for GitHub storage. Access via:

1. **Google Drive**: Request link via GitHub issue
2. **YouTube (unlisted)**: Contact @DKrishna007 for link  
3. **On-site**: Available for demonstration at UD Robotics Lab

## Screen Recording Instructions

```bash
# Launch system
ros2 launch smart_wheelchair_autonomy wheelchair_launch.py

# Record RViz2 screen (Linux)
ffmpeg -f x11grab -s 1920x1080 -i :0.0 -r 30 \
       -codec:v libx264 -preset fast \
       wheelchair_demo_$(date +%Y%m%d_%H%M%S).mp4

# Stop recording: Ctrl+C
```

## Key Visualization Topics

When running with RViz2:
- `/lidar_front/points` — Front VLP-16 point cloud
- `/lidar_rear/points` — Rear VLP-16 point cloud  
- `/map` — Cartographer SLAM map
- `/local_costmap/costmap` — Navigation costmap
- `/plan` — Global path
- `/local_plan` — Local trajectory
- `/cmd_vel` — Final velocity command (post-DQN)

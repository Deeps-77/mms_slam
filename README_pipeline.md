# Dynamic Warehouse RGB-D Dataset Pipeline

## Overview
This pipeline provides ready-to-run launch files for the dynamic_warehouse RGB-D dataset with Point-BERT loop closure integration.

## Files Added
- `src/republish_lidar_to_filtered_static.py` - XYZRGB republisher node
- `config/pointbert_params.yaml` - Point-BERT configuration parameters
- `launch/dynamic_warehouse_pointbert_mapping.launch` - Main launch file
- `rviz/dynamic_warehouse_pointbert.rviz` - Visualization configuration

## Usage

### Option 1: Using the republisher (for clouds without RGB field)
```bash
# Set Point-BERT parameters (edit paths for your system)
rosparam set /steal_pointbert/pointbert/repo_path "/path/to/Point-BERT"
rosparam set /steal_pointbert/pointbert/checkpoint "/path/to/point-bert.pth"
rosparam set /steal_pointbert/pointbert/dvae_ckpt "/path/to/dvae.pth"

# Launch the pipeline
roslaunch mms_slam dynamic_warehouse_pointbert_mapping.launch bag:=/path/to/dynamic_warehouse.bag play_rate:=1.0 use_solo_dynamic_filter:=false
```

### Option 2: Using SOLO dynamic filtering
```bash
roslaunch mms_slam dynamic_warehouse_pointbert_mapping.launch bag:=/path/to/dynamic_warehouse.bag play_rate:=1.0 use_solo_dynamic_filter:=true
```

### Standalone republisher usage
```bash
rosrun mms_slam republish_lidar_to_filtered_static.py _input:=/camera/depth/color/points _output:=/filtered_points_static _frame_id:=base_link _rgb:=[255,255,255]
```

## Parameters
- `bag`: Path to the rosbag file (default: `/mnt/d/dynamic_warehouse.bag`)
- `play_rate`: Playback rate (default: `1.0`)
- `use_solo_dynamic_filter`: Use SOLO filtering instead of simple relay (default: `false`)
- `rgbd_topic`: Input RGB-D point cloud topic (default: `/camera/depth/color/points`)
- `frame_id`: Output frame ID (default: `base_link`)

## Visualization
The RViz configuration includes:
- Static point clouds (`/filtered_points_static`)
- Filtered lidar points (`/laser_points_filtered`) 
- Edge points (`/laser_cloud_edge`)
- Surface points (`/laser_cloud_surf`)
- Trajectory (`/mms_slam/trajectory`)
- Loop closure candidates (`/steal/loop_candidates`)
- Verified loop closures (`/steal/loop_verified`)
# tree_mapper

`tree_mapper` is a ROS 2 package for vegetation candidate detection, classification, tracking, and map fusion from point clouds.

It is planner-agnostic and it can be used with any stack that provides:

- a `sensor_msgs/PointCloud2` topic
- valid TF from cloud frame to target frame

## What It Does

- Extracts multi-slice woody vegetation candidates from point clouds (`vegetation_candidate_detector.py`)
- Classifies each candidate into `tree` or `shrub` (`vegetation_candidate_classifier.py`)
- Tracks classified detections over time (`object_id_tracker.py`)
- Fuses tracked detections into stable deduplicated object maps (`object_map_fuser.py`)

The detector supports both circle-fitting paths:

- `fit_method: ransac`
- `fit_method: rlts`

The detector is still geometry-first: it combines circle evidence across slices and falls back to PCA-shaped blob candidates for shrub-like vegetation.

## Minimal Integration Contract

### Required input

- `vegetation_candidate_detector/input_cloud_topic` (`sensor_msgs/PointCloud2`)
- `vegetation_candidate_detector/target_frame` (TF target frame)

### Main outputs

- `tree_map_fuser/output_topic` (`geometry_msgs/PoseArray`)
- `tree_map_fuser/output_id_topic` (`std_msgs/Int32MultiArray`)
- `tree_map_fuser/output_array_topic` (`tree_mapper/ObjectDetectionArray`)
- `tree_map_fuser/marker_topic` (`visualization_msgs/MarkerArray`)

## Interface YAML

Example file:

- `tree-ws/src/tree_mapper/config/interface_topics.yaml`

Typical fields:

```yaml
vegetation_candidate_detector:
  ros__parameters:
    input_cloud_topic: /uav1/os_cloud_nodelet/points
    target_frame: uav1/ground_truth_origin
    fit_method: ransac
```

Current `interface_topics.yaml` defaults use the raw cloud on `/uav1/os_cloud_nodelet/points`,
transform detections into `uav1/ground_truth_origin`, and enable a near-body self-filter so the detector ignores points from the UAV itself.

## Launch

Build with `colcon` before launching:

```bash
colcon build --packages-select tree_mapper
source install/setup.bash
```

Default interface launch:

```bash
ros2 launch tree_mapper tree_stack_interface.launch.py
```

Recommended launch:

```bash
ros2 launch tree_mapper tree_stack_interface.launch.py \
  config_yaml:=/absolute/path/to/teste.yaml
```

When using the tmux simulation in `ros2-ws/tmux/gazebo`, keep `--enable-ground-truth`
enabled in the drone spawn command so `uav1/ground_truth_origin` is available to the detector.

Custom interface YAML:

```bash
ros2 launch tree_mapper tree_stack_interface.launch.py config_yaml:=/absolute/path/to/interface_topics.yaml
```

Enable live plot:

```bash
ros2 launch tree_mapper tree_stack_interface.launch.py run_plotter:=true
```

Enable both real-time panels:

```bash
ros2 launch tree_mapper tree_stack_interface.launch.py run_plotter:=true run_cluster_plotter:=true
```

Both plotters can show the drone route/head using a `nav_msgs/Odometry` topic configured in interface YAML:

- `tree_xy_plotter.route_pose_topic`
- `tree_cluster_xy_plotter.route_pose_topic`

The cluster plotter shows the robot's current instantaneous cluster/detection state and draws estimated diameters for:

- current visible objects
- confirmed map trees
- candidate map trees

Current flow:

- `cluster_points` / `cluster_labels`: all candidate support points stacked across accepted slices
- `detections_full`: classified tree or shrub detections
- `tracked_full`: persistent tracked detections
- `tree_map_full` / `shrub_map_full`: final deduplicated confirmed maps

It can also overlay world ground truth from a CSV with different colors for `tree` and `bush`.
Ground truth is rendered as outline-only markers in the cluster panel so it stays readable over live clusters.

## Manual Export (Map History)

`tree_map_fuser` supports a manual export trigger (CSV/JSON):

```bash
ros2 topic pub --once /tree_map_fuser/export_now std_msgs/msg/Empty "{}"
```

The trigger topic is configurable in interface YAML:

- `tree_map_fuser.export_now_topic`

## Notes

- If no map is published, check TF first (cloud frame must resolve to `target_frame`).
- If the drone looks visually offset from detections in the plotters, compare the route topic frame with the detection/map frame; the plotters now warn when they differ.

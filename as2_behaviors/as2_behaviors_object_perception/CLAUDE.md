# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Package Overview

`as2_behaviors_object_perception` is a ROS 2 behavior server for object perception in autonomous aerial systems (Aerostack2 framework). It exposes an `as2_msgs/action/DetectObjects` action server and implements a modular **4-stage pipeline**:

1. **Image Preprocessing** — centralized in `PerceptionBehavior` (decompression + optional rectification via `ImagePreprocessor`)
2. **Object Detection** — pluggable via `detection_plugin_base::DetectionBase` (implemented: `detector_yolo_gates`)
3. **Object Identification** — run as a separate `PerceptionBehavior` instance consuming the detector output (implemented: `tracker_tf`)
4. **Object Pose Estimation** — future stage (not yet implemented)

Stages 1–2 run intraprocess (single node). Stage 3 (`tracker_tf`) runs as a separate node and subscribes to the `ObjectPerceptionArray` topic published by stage 2.

## Build & Development Commands

Build the package (from workspace root `/home/alba/PRACTICAS/aerostack2_ws`):
```bash
# Build as2_msgs first when message types change
colcon build --packages-select as2_msgs
colcon build --packages-select as2_behaviors_object_perception
source install/setup.bash
```

Run with the launch file:
```bash
ros2 launch as2_behaviors_object_perception detect_behavior_launch.py namespace:=drone0 use_sim_time:=true
```

Run directly:
```bash
ros2 run as2_behaviors_object_perception as2_behaviors_object_perception_node \
  --ros-args \
  -p plugin_name:=detector_yolo_gates \
  -p camera_image_topic:=/sensor_measurements/camera/image/compressed \
  -p camera_info_topic:=/sensor_measurements/camera/camera_info \
  -p enable_rectification:=false
```

## Architecture

### Pipeline Data Flow

**Stage 2 — Detection (`detector_yolo_gates`)**
```
CompressedImage → PerceptionBehavior::image_callback()
                    → ImagePreprocessor (decompress + optional rectify)
                    → DetectionBase::image_callback(cv::Mat, Header)

action::on_run() → DetectionBase::own_run()
                     → YOLO inference (throttled at inference_frequency Hz)
                     → write results to latest_detections_
                  → PerceptionBehavior reads getDetections()
                  → publish ObjectPerceptionArray (debug topic)
                  → populate action feedback/result
```

**Stage 3 — Identification (`tracker_tf`)**
```
ObjectPerceptionArray  →  detectionsCallback()  →  raw_detections_
(from detector_yolo_gates)

action::on_run() → own_run()
                     → TF lookup: map → camera_frame  (once per cycle)
                     → for each known gate: projectToImage() → pixel (u, v)
                     → for each raw detection: bbox centre → matchGate()
                     → write identified detections to latest_detections_
                  → PerceptionBehavior reads getDetections()
                  → populate action feedback/result with ObjectPerceptionArray
                    (pose_valid=true, pose = known gate pose from config)
```

### Detection Plugin Base (`detection_plugin_base.hpp`)

All detection plugins inherit from `detection_plugin_base::DetectionBase`:

- `image_callback(cv::Mat, Header)` — receives pre-processed frame (no decompression needed in plugin)
- `camera_info_callback(CameraInfo)` — default impl extracts `camera_matrix_` / `dist_coeffs_`
- `own_run()` — throttled inference loop; writes to `latest_detections_` (inherited member)
- `getDetections()` — called by `PerceptionBehavior` to populate action feedback
- `own_activate/modify/deactivate/pause/resume()` — behavior lifecycle hooks

### Adding a New Detection Plugin

1. Create `plugins/my_detector/` with `include/my_detector/my_detector.hpp` and `src/my_detector.cpp` inheriting `detection_plugin_base::DetectionBase`.
2. Register in `plugins.xml`:
   ```xml
   <class type="my_detector::Plugin" name="my_detector::Plugin"
          base_class_type="detection_plugin_base::DetectionBase">
     <description>...</description>
   </class>
   ```
3. Add to `PLUGIN_LIST` in `CMakeLists.txt` and add a library target following the pattern of `detector_yolo_gates`.

### Common Utilities (`include/as2_behaviors_object_perception/common/`)

- `img_preprocessing.hpp / .cpp` — used by `PerceptionBehavior`, NOT by plugins. Handles JPEG/PNG decompression and CPU/CUDA distortion rectification with cached maps.
- `common.hpp` — `MutexQueue<T>` (thread-safe drop queue), `TimerProcessor<T>` (rate-limited callback), pose estimation data structures.
- `arducam_interface.hpp / .cpp` — hardware camera interface (optional, Jetson-specific).

### Message Types (as2_msgs)

- `as2_msgs/action/DetectObjects` — new action replacing `Detect`; feedback and result carry `ObjectPerceptionArray`
- `as2_msgs/msg/ObjectPerception` — single detected object: `id`, `class_name`, `confidence`, `pose` (PoseStamped, valid when `pose_valid=true`), `keypoints` (2D image coords, z=0), `keypoint_names`, `keypoint_scores`, `bbox_min/max`, `flags`
- `as2_msgs/msg/ObjectPerceptionArray` — `header` + `ObjectPerception[]`

## Configuration

Default parameters are in `config/config_default.yaml`. Key top-level parameters:

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `plugin_name` | `"detector_yolo_gates"` | Which detection plugin class to load |
| `persistent` | `true` | Keep action running after first detection cycle |
| `enable_rectification` | `false` | Enable distortion correction (PerceptionBehavior level) |
| `camera_image_topic` | `/sensor_measurements/camera/image/compressed` | Compressed image input |
| `camera_info_topic` | `/sensor_measurements/camera/camera_info` | Camera calibration |

Plugin-specific parameters live under a namespace matching the plugin directory name (e.g., `detector_yolo_gates.*`).

## ROS 2 Interfaces

Both plugins share the same `PerceptionBehavior` node structure:

- **Action server**: `as2_msgs/action/DetectObjects`
- **Subscriptions (PerceptionBehavior)**: `sensor_msgs/msg/CompressedImage`, `sensor_msgs/msg/CameraInfo`
- **Publishers (`detector_yolo_gates`, when `enable_debug: true`)**: `as2_msgs/msg/ObjectPerceptionArray` (raw detections), `sensor_msgs/msg/CompressedImage` (annotated image)
- **Subscriptions (`tracker_tf`)**: `as2_msgs/msg/ObjectPerceptionArray` (raw detections from `detector_yolo_gates`)
- **Publishers (`tracker_tf`)**: `as2_msgs/msg/ObjectPerceptionArray` (identified detections: matched gate name, known pose, all YOLO keypoints/bbox forwarded)

Topics are namespaced with the node namespace. QoS follows `as2_names::topics::sensor_measurements::qos`.

## Plugin: `tracker_tf`

Gate identification plugin that consumes raw 2D detections from `detector_yolo_gates` and identifies each detected gate by **reverse projection**: all known gate positions (from a YAML config file) are projected into the image using the current camera pose from TF, and each detection is matched to the nearest projected gate. Runs as a **separate `PerceptionBehavior` instance** (not chained with the detector in the same node).

No pose estimation is performed — the known ground-truth pose from the config is used directly in the output.

### Data flow

```
detector_yolo_gates  →  ObjectPerceptionArray topic  →  tracker_tf::detectionsCallback()
(2D keypoints, pose_valid=false)                              ↓
                                                        own_run() per cycle:
                                                          TF: map → camera_frame  (once)
                                                          for each known gate:
                                                            projectToImage()
                                                              → 2D pixel position
                                                          for each raw detection:
                                                            bbox centre (u, v)
                                                            matchGate() nearest projected gate
                                                              within max_pixel_dist px
                                                          emit ObjectPerception:
                                                            id/class_name = gate name
                                                            pose          = known pose (map)
                                                            pose_valid    = true
```

### Reverse projection (`projectToImage`)

```
gate (x,y,z) in map
    ↓  TransformStamped: map → camera_frame
gate (xc, yc, zc) in camera frame   [skip if zc ≤ 0: behind camera]
    ↓  pinhole projection
u = fx · xc/zc + cx
v = fy · yc/zc + cy
```

The camera intrinsics (`fx, fy, cx, cy`) come from `camera_matrix_` (populated by the base-class `camera_info_callback`). No gate dimensions are needed.

### Gate config file format (`plugins/tracker_tf/config/gates_config.yaml`)

```yaml
gates_poses:
  gate01: [x, y, z, yaw]   # metres / radians in map frame
  gate02: [x, y, z, yaw]
  ...
```

### Parameters (`tracker_tf.*`)

| Parameter | Default | Purpose |
|---|---|---|
| `detections_topic` | `""` | `ObjectPerceptionArray` topic from `detector_yolo_gates` |
| `output_topic` | `""` | Topic to publish identified detections (pose + all YOLO data) |
| `gates_config_file` | `""` | Absolute path to gate poses YAML |
| `max_pixel_dist` | `150.0` | Max pixel distance to accept a gate match |
| `reference_frame` | `"map"` | World frame for output poses |
| `tf_timeout` | `0.1` | TF lookup timeout (s) |

Sample parameter file: `config/config_tracker_tf.yaml`.

### Notes

- `detector_yolo_gates` must have `enable_debug: true` and `detections_data_topic` set so it publishes the `ObjectPerceptionArray` that `tracker_tf` subscribes to.
- New CMake/package.xml dependencies added: `tf2`, `tf2_ros`, `tf2_geometry_msgs`, `yaml_cpp_vendor`.

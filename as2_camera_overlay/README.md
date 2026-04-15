# as2_camera_overlay

A headless ROS 2 node that projects RViz-style 3D visualizations (markers, grids, paths, TF frames, etc.) directly into camera images and publishes the augmented images as sensor streams.

## Overview

`as2_camera_overlay` is an Aerostack2 component that enables augmented reality (AR) visualization for autonomous drone systems. It takes a camera image stream and overlays relevant 3D visualization elements onto the image, then publishes the augmented result. This is particularly useful for:

- **Real-time Mission Monitoring**: Visualize flight paths, waypoints, and obstacles directly in the camera feed
- **Headless Operation**: Runs without GUI, ideal for embedded systems and autonomous platforms

## Features

- ✨ **Multiple Visualization Types**: Grid, MarkerArray, Path, PoseArray, PoseStamped, TF frames
- 🎨 **GPU-Accelerated Rendering**: Uses Ogre 3D engine for efficient render-to-texture operations
- 🔌 **Plugin Architecture**: Easily extensible with custom display types
- 📸 **Flexible Camera Support**: Works with any camera providing standard ROS 2 image/camera_info topics
- 🗺️ **Frame-Based Transformations**: Automatically handles TF lookups and coordinate transformations

## Dependencies

### Core Dependencies
- **OpenCV**: For image processing
- **Eigen3**: For linear algebra
- **Ogre 3D**: Via `rviz_ogre_vendor` and `rviz_rendering`

## Quick Start

### 1. Basic Launch

Launch the camera overlay node with default configuration:

```bash
ros2 launch as2_camera_overlay camera_overlay.launch.py
```

### 2. Custom Configuration

Launch with a custom parameter file:

```bash
ros2 launch as2_camera_overlay camera_overlay.launch.py \
  config:=/path/to/your/config.yaml
```

## Configuration

Configuration is managed via a YAML parameter file. Default configuration is located at `config/defaults.yaml`.

### Parameter Reference

#### Global Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `fixed_frame` | string | `"earth"` | Reference frame for fixed-coordinate displays. |
| `near_plane` | float | `0.05` | Near clipping plane distance (m) |
| `far_plane` | float | `1000.0` | Far clipping plane distance (m) |
| `zoom_factor` | float | `1.0` | Camera zoom factor for perspective adjustment |
| `render_scale` | float | `0.5` | Render resolution scale (for preformace) |
| `max_render_fps` | float | `0.0` | Maximum render FPS (0 = unlimited) |

#### Input Topics

| Parameter | Type | Description |
|-----------|------|-------------|
| `input.image_topic` | string | Input camera image stream (sensor_msgs/Image) |
| `input.camera_info_topic` | string | Camera info (sensor_msgs/CameraInfo) |

#### Output Topics

| Parameter | Type | Description |
|-----------|------|-------------|
| `output.topic` | string | Output augmented image stream (sensor_msgs/Image) |

#### Display Configuration

| Parameter | Type | Description |
|-----------|------|-------------|
| `enabled_displays` | list | List of display plugins to load |
| `displays.<DisplayName>` | dict | Plugin-specific configuration (see **Available Displays** section) |

### Example Configuration

see `config/defaults.yaml`

## Available Displays

Displays are loaded as plugins. Each display renders a specific ROS 2 message into the camera image.

### GridDisplay

Renders a 2D grid plane in the 3D scene.

**Parameters:**
- `reference_frame` (string): Frame in which to place the grid
- `plane` (string): Grid plane orientation - `"XY"`, `"XZ"`, or `"YZ"`
- `cell_count` (int): Number of grid cells (in each direction)
- `cell_size` (float): Size of each grid cell (m)
- `line_width` (float): Line thickness (m)
- `color_rgba` (list): RGBA color `[R, G, B, A]` 

**Example:**
```yaml
displays:
  GridDisplay:
    reference_frame: earth
    plane: XY
    cell_count: 25
    cell_size: 1.0
    line_width: 0.02
    color_rgba: [0.5, 0.5, 0.5, 1.0]
```

### MarkerArrayDisplay

Renders ROS visualization_msgs/MarkerArray messages.

**Parameters:**
- `topics` (list): ROS topics to subscribe to (e.g., `["gates_tracking", "gates_static"]`)
- `queue_size` (int): Message queue size for each topic

**Example:**
```yaml
displays:
  MarkerArrayDisplay:
    topics: ["gates_static", "gates_tracking"]
    queue_size: 10
```

**Input Message Type:** `visualization_msgs/MarkerArray`

### see `src/displays` for more displays

## Architecture

### High-Level Design

```
Camera Image + Camera Info
         ↓
    OverlayNode (ROS 2 Node)
         ├─ CameraProjection (camera intrinsics & projection)
         ├─ OverlayRenderer (Ogre render target)
         │   └─ Display Plugins (GridDisplay, MarkerArrayDisplay, etc.)
         └─ TF2 Buffer (transform lookups)
         ↓
   Augmented Image
```

### Plugin Architecture

Displays are implemented as plugins using `pluginlib`. To create a custom display:

1. Inherit from `as2_camera_overlay::OverlayDisplayBase`
2. Implement `onInitialize()` and `update()` methods
3. Use the Ogre SceneManager to create visual elements
4. Register in `plugins.xml`

## License

This package is part of Aerostack2.

See the main Aerostack2 repository for full license details.


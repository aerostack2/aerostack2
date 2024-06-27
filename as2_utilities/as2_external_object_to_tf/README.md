# as2_external_object_to_tf

This node is used to add an external object to TF tree using Aerostack2.

There are several data sources that we can use to add an object to the TF tree. 

- **PoseStamped topic**
- **NavSatFix topic** + **Azimuth topic**
- **Static transform**: The transform data will be defined either in the configuration file, or the services that allow it.

The following example configuration file defines every possible way of adding an object to the TF tree:

``` yaml
# YAML
---
objects:
  - type: pose
    parent_frame: earth
    frame: object_0
    pose_topic: "/object_0/pose"
  - type: gps
    parent_frame: earth
    frame: object_1
    gps_topic: "/object_1/sensor_measurements/gps"
    azimuth_topic: "/object_1/gps/azimuth"
  - type: gps_static
    parent_frame: earth
    frame: object_2
    gps_pose:
      lat: 52.171775999796274
      lon: 4.416570248787041
      alt: 50.520017601549625
    orientation:
      azimuth: 180.0
      bank: 0.0
      elevation: 0.0
  - type: pose_static
    frame: object_3
    pose:
      position:
        x: 5.0
        y: 0.0
        z: 0.0
      orientation:
        r: 0.0
        p: 0.0
        y: 1.57
  - type: mocap
    rigid_bodies: 
      - rigid_body: gate_0
        frame: object_0
      - rigid_body: gate_1
        frame: object_0
```
Aditionally, static transformations can be added to the TF tree via services, both defined by a pose or gps data.

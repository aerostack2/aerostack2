# as2_python_api

AeroStack2 python interface.

## Index
- [API](#api)
- [ENUMERATIONS](#enumerations)
    - [STATE](#state)
    - [YAW_MODE](#yawmode)
    - [CONTROL_MODE](#controlmode)
    - [REFERENCE_FRAME](#referenceframe)

----
## API

| Method | Input | Output | Description |
| --- | --- | --- | --- |
| `str = get_drone_id()` | - | drone namespace | - |
| `{ bool, bool, bool, STATE, YAW_MODE, CONTROL_MODE, REFERENCE_FRAME } = get_info()` | - | drone info {connected, armed, offboard, state, yaw_mode, control_mode, reference_frame} | - |
| `[ float, float, float ] = get_position()` | - | [ x, y, z ] | - |
| `[ float, float, float ] = get orientation()` | - | [ roll, pitch, yaw ] | - |
| `[ float, float, float ] = get_gps_pose()` | - | [ lat, lon ,alt ] |
| `takeoff(float, float)` | heigth, speed | - | - |
| `follow_path([ [ float, float, float ], ... ], float)` | [ [ x, y, z ], ... ], speed | - | - |
| `follow_gps_path([ [ float, float, float ], ... ], float)` | [ [ lat, lon, alt ], ... ], speed | - | - |
| `land()` | - | - | - |
<!-- | `go_to()` | - | - | -->

----
## ENUMERATIONS

### STATE

| Value | Field Name | Description |
| --- | --- | --- |
| `EMERGENCY` | -1 | - |
| `DISARMED` | 0 | - |
| `LANDED` | 1 | - |
| `TAKING_OFF` | 2 | - |
| `FLYING` | 3 | - |
| `LANDING` | 4 | - |

### YAW_MODE

| Value | Field Name | Description |
| --- | --- | --- |
| `YAW_ANGLE` | 0 | - |
| `YAW_SPEED` | 1 | - |

### CONTROL_MODE

| Value | Field Name | Description |
| --- | --- | --- |
| `UNSET` | -1 | mode when the vehicle is not set |
| `POSITION_MODE` | 0 | x, y, z refs |
| `SPEED_MODE` | 1 | vx, vy, vz refs |
| `SPEED_IN_A_PLANE` | 2 | vx, vy, z refs |
| `ACCEL_MODE` | 3 | ax, ay, az refs |
| `ATTITUDE_MODE` | 4 | quaternions + Thrust  |
| `ACRO_MODE` | 5 | p, q, r speed + Thrust  |

### REFERENCE_FRAME

| Value | Field Name | Description |
| --- | --- | --- |
| `LOCAL_ENU_FRAME` | 0 | Local coordinates |
| `BODY_FLU_FRAME` | 1 | Body coordinates |
| `GLOBAL_ENU_FRAME` | 2 | GPS coordinates |
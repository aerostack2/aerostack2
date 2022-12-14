# gazebo_assets
Colletion of AS2 Gazebo assets and scripts.

## INDEX
- [HOW TO RUN](#how-to-run-basic-usage)
- [OPTIONS](#options)
    - [ENV VARS](#env-vars)
    - [CONFIG FILE](#config-file)
- [MORE OPTIONS](#more-options)
- [EXAMPLES](#examples)
- [ADVANCED USAGE](#advanced-usage)
---

## HOW TO RUN: Basic usage

Previously setting AS2 environment, simply run:
```bash
${AEROSTACK2_PATH}/as2_simulation_assets/gazebo_assets/scripts/default_run.sh 
```

or using a config file (see [config files](#config-file)) :

```bash
${AEROSTACK2_PATH}/as2_simulation_assets/gazebo_assets/scripts/default_run.sh <config-file>
```

This will run for you **gzserver**, spawn an **iris model**, compile and run **PX4 SITL rtps** and open **gzclient**.

## OPTIONS
Inital configuration aspects as world, drone model, drone pose or adding several drones can be done setting **environment variables** or using a **config file**.

### ENV VARS
Previously set needed environment variables before launching the script.

- World
    ```bash
    export PX4_SITL_WORLD=<path-to-world>
    ```
- Drone model
    ```bash
    export UAV_MODEL=<model-name>
    ```
- Drone pose
    ```bash
    export UAV_X=<float>  # meters
    export UAV_Y=<float>  # meters
    export UAV_Z=<float>  # meters
    export UAV_YAW=<float>  # radians
    ```

### CONFIG FILE
Using a config file lets you to set the simulation environment. You can select a world (or none) and attach to it a number of desired drones with desired model and position. Please pay atention to teh format file, otherwise it may fail.

Example of a config file:
```json
{
    "world": "${AEROSTACK2_PATH}/as2_simulation_assets/gazebo_assets/worlds/frames.world",
    "0": {
        "model": "iris_fpv",
        "pose": [ 0.0, 0.0, 0.0, 1.57 ]
    },
    "1": {
        "model": "iris",
        "pose": [ 3.0, 0.0, 0.0, 1.57 ]
    }
}
```

## MORE OPTIONS
- Use custom models in world/drone:
    ```bash
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<custom-model-path>
    ```
- Run simulation without gzclient (TBD):
    ```bash
    export HEADLESS=1
    ```
- Follow drone mode (TBD):
    ```bash
    export PX4_FOLLOW_MODE=1
    ```
- Change PX4 GPS origin:

    ```bash
    export PX4_HOME_LAT=28.143971  # degrees
    export PX4_HOME_LON=-16.503213  # degrees
    export PX4_HOME_ALT=0  # meters
    ```
- Change PX4 vehicle (TBD):
    ```bash
    export VEHICLE=iris  # typhoon
    ```
- Verbose mode:
    ```bash
    export VERBOSE_SIM=1
    ```

## EXAMPLES
Several examples can be found on [test](/tests) folder.

## ADVANCED USAGE

If you want to build your own PX4 first, you can also check [test_run_sitl](/tests/test_run_sitl.sh) script.

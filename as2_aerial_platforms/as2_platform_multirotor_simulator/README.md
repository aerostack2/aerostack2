# as2_platform_multirotor_simulator

Aerostack2 lightweight simulator for multirotor drones, based on [multirotor_simulator](https://github.com/RPS98/multirotor_simulator.git).

## API
For documentation, use [Doxygen](https://www.doxygen.nl/index.html). The documentation files have to be built locally:

1. Install doxygen:
    ```
    sudo apt install doxygen
    ```
2. Run doxygen in the root folder `as2_platform_multirotor_simulator/` by:
    ```
    doxygen Doxyfile
    ```
3. Open the documentation in by opening `as2_platform_multirotor_simulator/doxygen/html/index.html`

## Building the package

Go to the root folder of the workspace and run:
```
colcon build --packages-select as2_platform_multirotor_simulator
```

## Running colcon test

Run colcon test in by:
```
colcon test --packages-select as2_platform_multirotor_simulator
```
   
Optional, add verbosity with `--event-handlers console_direct+`


## Launching the simulator

Configuration files are located in `as2_platform_multirotor_simulator/config/`:
* `control_modes.yaml`: available input control modes for the aerostack2 aerial platform.
* `platform_config_file.yaml`: ROS 2 configuration, with tf names, frequencies and aerostack2 parameters.
* `simulation_config.yaml`: simulation configuration, with frequency and environment parameters.
* `uav_configuav_config.yaml`: UAV configuration, with the drone's physical parameters and controller configuration.
* `world_config.yaml`: world configuration, as parameters override for each drone namespace in the configuration file.

Launcher parameters order
1. Default configuration file in `as2_platform_multirotor_simulator/config/` folder.
2. Custom configuration file in the launch command (e.g. `platform_config_file:=path/to/file.yaml`).
3. Custom parameters in the launch command (e.g. `frequency:=100`).
4. Custom world configuration in the launch command (e.g. `world_config:=path/to/file.yaml`), for the world launcher.

Examples:
```
ros2 launch as2_platform_multirotor_simulator as2_platform_multirotor_simulator.launch.py namespace:=drone0
```
```
ros2 launch as2_platform_multirotor_simulator as2_platform_multirotor_simulator_world.launch.launch.py namespace:=drone0 world_config:=config/world_config.yaml
```




## References

M. Fernandez-Cortizas, M. Molina, P. Arias-Perez, R. Perez-Segui, D. Perez-Saura, and P. Campoy, 2023,  ["Aerostack2: A software framework for developing multi-robot aerial systems"](https://arxiv.org/abs/2303.18237), ArXiv DOI 2303.18237.

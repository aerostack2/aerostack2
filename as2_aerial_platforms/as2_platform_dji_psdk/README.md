# as2_platform_dji_psdk

Aerostack2 DJI PSDK platform package.

## Dependencies

Follow the [psdk_ros2 installation guide](https://umdlife.github.io/psdk_ros2/documentation/GettingStarted.html)

## API
For documentation, use [Doxygen](https://www.doxygen.nl/index.html). The documentation files have to be built locally:

1. Install doxygen:
    ```
    sudo apt install doxygen
    ```
2. Run doxygen in the root folder `as2_platform_dji_psdk/` by:
    ```
    doxygen Doxyfile
    ```
3. Open the documentation in by opening `as2_platform_dji_psdk/doxygen/html/index.html`

## Building the package

Go to the root folder of the workspace and run:
```
colcon build --packages-select as2_platform_dji_psdk
```

## Running colcon test

Run colcon test in by:
```
colcon test --packages-select as2_platform_dji_psdk
```
   
Optional, add verbosity with `--event-handlers console_direct+`

## References

M. Fernandez-Cortizas, M. Molina, P. Arias-Perez, R. Perez-Segui, D. Perez-Saura, and P. Campoy, 2023,  ["Aerostack2: A software framework for developing multi-robot aerial systems"](https://arxiv.org/abs/2303.18237), ArXiv DOI 2303.18237.

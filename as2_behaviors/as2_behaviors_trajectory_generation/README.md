# as2_behaviors_trajectory_generation

## External dependencies

The following upstream libraries are vendored under each plugin's
`thirdparties/` directory and fetched automatically at CMake configure
time (`find_package` → local clone → `FetchContent`). The directories
are gitignored and never committed to this repository.

- Dynamic Trajectory Generator: <https://github.com/cvar-upm/dynamic_trajectory_generator>, based on <https://github.com/ethz-asl/mav_trajectory_generation>
- Mav Trajectory Generation: <https://github.com/aerostack2/mav_trajectory_generation_library>, based on <https://github.com/ethz-asl/mav_trajectory_generation>
- GCOPTER Trajectory Generator: <https://github.com/aerostack2/gcopter_trajectory_generator_lib>, based on <https://github.com/ZJU-FAST-Lab/GCOPTER>
- Jerk-Limited Trajectory Generator: <https://github.com/aerostack2/jerk_limited_trajectory_generator_lib>, based on <https://github.com/PX4/PX4-Autopilot>
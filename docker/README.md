# Docker
```
make -f docker/Makefile build
```

## Using GPU 

Install `nvidia-cuda-toolkit` on host machine.

Add `--gpu=all` and `--device /dev/dri` flags to `docker run` command.

## Enable GUI apps visualization
TODO

## How to run with development project

From your development project folder, run your container mounting the volume with `-v $(pwd):/home/cvar/my_project:rw` flag. Example:
```
docker run -it --gpus=all \
    --device /dev/dri/ \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -v $(pwd):/home/cvar/my_project:rw \
    as2:humble
```

## Dockerhub

Images are available at dockerhub `cvarupm/aerostack2` with tags:
- `rolling`: source built aerostack2 `main` branch on more recent LTS ros distribution.
- `humble`: latest official aerostack2 release on ros humble distribution.
- `galactic` (aka `galactic-rolling`): source built aerostack2 `main` branch on ros galactic distribution

## Known issues

- Galactic build not working:
```
 --- stderr: as2_ign_gazebo_assets
 CMake Error at CMakeLists.txt:44 (find_package):
   By not providing "Findros_gz_sim.cmake" in CMAKE_MODULE_PATH this project
   has asked CMake to find a package configuration file provided by
   "ros_gz_sim", but CMake did not find one.
```

- `as2_cli` source failing
```
/home/cvar/aerostack2_ws/src/aerostack2/as2_cli/setup_env.bash: line 22: /home/cvar/aerostack2_ws/src/aerostack2/as2_cli/env_variables.bash: Permission denied
```

- GUI visualization: might not work in other hosts
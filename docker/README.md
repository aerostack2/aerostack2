# Docker

Aerostack2 container images are managed with Docker Compose (`docker/compose.yaml`).
The same commands are also available as `pixi` tasks (e.g. `pixi run docker:build-humble`).

## Build

Build all images (`humble`, `nightly-humble`, `jazzy`):

```
docker compose -f docker/compose.yaml build
```

Build a single image:

```
docker compose -f docker/compose.yaml build humble
```

Ignore the build cache for a clean rebuild:

```
docker compose -f docker/compose.yaml build --no-cache humble
```

## Pull

Pull the published images from Docker Hub:

```
docker compose -f docker/compose.yaml pull humble nightly-humble
```

## Run

```
docker run -it --rm aerostack2/humble
```

## Using GPU

Install `nvidia-cuda-toolkit` on host machine.

Add `--gpu=all` and `--device /dev/dri` flags to `docker run` command.

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
    aerostack2/humble
```

## Clean

Remove the images built locally:

```
docker compose -f docker/compose.yaml down --rmi all
```

## Dockerhub

Images are available at Docker Hub under the `aerostack2` organization (<https://hub.docker.com/u/aerostack2>):

- `aerostack2/humble` (`latest`, plus version tags such as `1.1.3`): latest official aerostack2 release on ROS humble.
- `aerostack2/nightly-humble` (`latest`, plus a tag per commit): source build of aerostack2 on every push to `main`.

Supported ROS distributions are **humble** and **jazzy**. The `jazzy` image is not published yet; build it locally (from the current repository checkout) with:

```
docker compose -f docker/compose.yaml build jazzy
```

## Known issues

- `as2_cli` source failing
```
/home/cvar/aerostack2_ws/src/aerostack2/as2_cli/setup_env.bash: line 22: /home/cvar/aerostack2_ws/src/aerostack2/as2_cli/env_variables.bash: Permission denied
```

- GUI visualization: might not work in other hosts

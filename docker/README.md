# Docker

Aerostack2 container images are managed with Docker Compose (`docker/compose.yaml`).
The same commands are also available as `pixi` tasks (e.g. `pixi run docker:build-humble`).

Both `humble` and `jazzy` run as `root`. The workspace is `/root/aerostack2_ws` and
`AEROSTACK2_PATH` is `/root/aerostack2_ws/src/aerostack2`.

`humble` and `nightly-humble` share `docker/humble/Dockerfile`. The distinction is only the
image tag: CI publishes `aerostack2/humble` on GitHub releases and `aerostack2/nightly-humble`
on every push to `main`.

## Build

Build the default images (`humble`, `jazzy`):

```
docker compose -f docker/compose.yaml build
```

Build a single image:

```
docker compose -f docker/compose.yaml build humble
```

`nightly-humble` is the same Dockerfile as `humble` under a different tag. It lives behind the
`nightly` compose profile so a default build does not compile it twice:

```
docker compose -f docker/compose.yaml --profile nightly build nightly-humble
```

Ignore the build cache for a clean rebuild:

```
docker compose -f docker/compose.yaml build --no-cache humble
```

## Pull

Pull the published images from Docker Hub:

```
docker compose -f docker/compose.yaml pull humble
docker compose -f docker/compose.yaml --profile nightly pull nightly-humble
```

## Run

`pixi run docker:run` is a smoke test (`docker run -it --rm aerostack2/humble`). For GPU access
or a mounted project, use `docker run` directly.

```
docker run -it --rm aerostack2/humble
```

## Using GPU

Install the NVIDIA Container Toolkit on the host.

Add `--gpus all` and `--device /dev/dri` to the `docker run` command.

## How to run with a development project

From your development project folder, mount it at `/root/my_project`:

```
docker run -it --rm --gpus all \
    --device /dev/dri \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -v $(pwd):/root/my_project:rw \
    aerostack2/humble
```

## Clean

Remove locally built or pulled images:

```
docker image rm -f aerostack2/humble aerostack2/nightly-humble aerostack2/jazzy
```

Or `pixi run docker:clean`.

## Docker Hub

Images are available at Docker Hub under the `aerostack2` organization
(<https://hub.docker.com/u/aerostack2>):

- `aerostack2/humble` (`latest`, plus version tags such as `1.1.3`): latest official aerostack2
  release on ROS Humble.
- `aerostack2/nightly-humble` (`latest`, plus a tag per commit): source build of aerostack2 on
  every push to `main`.

Supported ROS distributions are **humble** and **jazzy**. The `jazzy` image is not published yet;
build it locally (from the current repository checkout) with:

```
docker compose -f docker/compose.yaml build jazzy
```

## Legacy images

`docker/humble_vnc` and `docker/humble_gzharmonic` are not managed by `compose.yaml` and are
unsupported. Prefer the `humble` or `jazzy` images above.

## Known issues

- GUI visualization might not work on hosts other than the development machines.

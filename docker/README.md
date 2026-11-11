# Docker

Aerostack2 container images are managed with Docker Compose (`docker/compose.yaml`).

Both `humble` and `jazzy` run as `root`. The workspace is `/root/aerostack2_ws` and
`AEROSTACK2_PATH` is `/root/aerostack2_ws/src/aerostack2`.

All images build `docker/Dockerfile`, with the distro set by the `ROS_DISTRO` build arg. CI
publishes `aerostack2/<distro>` on GitHub releases and `aerostack2/nightly-<distro>` every
night when `main` has new commits.

## Build

Build the default images (`humble`, `jazzy`):

```
docker compose -f docker/compose.yaml build
```

Build a single image:

```
docker compose -f docker/compose.yaml build humble
```

`nightly-humble` and `nightly-jazzy` are the same builds as `humble` and `jazzy` under a
different tag. They live behind the `nightly` compose profile so a default build does not
compile them twice:

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
docker compose -f docker/compose.yaml pull humble jazzy
docker compose -f docker/compose.yaml --profile nightly pull nightly-humble nightly-jazzy
```

## Run

Start a shell in the image as a smoke test. Add flags for GPU access or a mounted project, as
shown below.

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
docker image rm -f aerostack2/humble aerostack2/nightly-humble aerostack2/jazzy aerostack2/nightly-jazzy
```

## Docker Hub

Images are available at Docker Hub under the `aerostack2` organization
(<https://hub.docker.com/u/aerostack2>), each for amd64 and arm64:

- `aerostack2/humble` and `aerostack2/jazzy` (`latest`, plus version tags such as `1.1.3`):
  latest official aerostack2 release.
- `aerostack2/nightly-humble` and `aerostack2/nightly-jazzy` (`latest`, plus a tag per commit):
  source build of `main`, rebuilt every night when it has new commits.

## Legacy images

`docker/humble_vnc` and `docker/humble_gzharmonic` are not managed by `compose.yaml` and are
unsupported. Prefer the `humble` or `jazzy` images above.

## Known issues

- GUI visualization might not work on hosts other than the development machines.

# Aerostack2 container

For building the image

```
docker build . -t aerostack2 --no-cache  --build-arg ROS_DISTRO=humble --build-arg AEROSTACK2_BRANCH=main
```

## Prerequisites for using GPU 

Install [`nvidia-cuda-toolkit`](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) on host machine.

Add `--gpu=all` and `--device /dev/dri` flags to `docker run` command.

## Deployment using docker-run

Runnning the container

```
docker run -it  \
    --gpus=all \
    --device /dev/dri/ \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
    -e XAUTHORITY=/tmp/.docker.xauth \
    aerostack2
```

## Deployment using docker-compose

For deploying the container
```
docker compose up -d 
# if this fail try 
docker-compose up -d 
```

For accessing the container (this can be used wherever not necesarily from this folder)

```
docker exec -it aerostack2 /bin/bash
```

For stopping the container
```
docker compose down
# if this fail try 
docker-compose down
```

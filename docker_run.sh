#!/bin/bash

xhost +local:docker

docker run -it \
    --privileged \
    --volume="/dev/:/dev/" \
    --network="host" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --gpus all \
    --runtime=nvidia \
    --env=NVIDIA_DRIVER_CAPABILITIES=all  \
    nikhil:thesis \
    bash

#!/bin/bash

# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.
## Build the image first
### docker build -t r2_path_planning .
## then run this script
xhost local:root

#    --volume /home/nikhil/Nikhil/Masters/Thesis/software/gpdhydra_ws:/root/gpdhydra_ws \
# XAUTH=/tmp/.docker.xauth

# if [ ! -f $XAUTH ]
# then
#     xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
#     if [ ! -z "$xauth_list" ]
#     then
#         echo $xauth_list | xauth -f $XAUTH nmerge -
#     else
#         touch $XAUTH
#     fi
#     chmod a+r $XAUTH
# fi

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
    uav-target-search \
    bash

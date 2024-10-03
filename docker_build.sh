#! /bin/bash

export DOCKER_BUILDKIT=1
docker build --ssh default -f Dockerfile  -t uav-target-search .

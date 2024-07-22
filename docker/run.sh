#!/bin/sh

usage() { echo "Usage: $0 [-t <tag>] [-i <image>]" 1>&2; exit 1; }

# Defaults
DOCKER_IMAGE_NAME="carla-ros-bridge"
TAG="gruppo1"

while getopts ":ht:i:" opt; do
  case $opt in
    h)
      usage
      exit
      ;;
    t)
      TAG=$OPTARG
      ;;
    i)
      DOCKER_IMAGE_NAME=$OPTARG
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
  esac
done
shift $((OPTIND-1))

echo "Using $DOCKER_IMAGE_NAME:$TAG"

docker run \
    -it --rm \
	--privileged \
    --net=host \
	--name gruppo1-ros-bridge-container \
	-e DISPLAY \
	-v /tmp/.X11-unix/:/tmp/.X11-unix \
	-v ~/.Xauthority:/root/.Xauthority \
	-e XAUTHORITY=/root/.Xauthority \
    "$DOCKER_IMAGE_NAME:$TAG" "$@"

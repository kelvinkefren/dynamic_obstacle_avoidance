#!/usr/bin/env bash

export CONTAINER_IMAGE="migbot_image"
export CONTAINER_NAME="migbot_container"
export WITH_NVIDIA="ON"
export DISABLE_X="1"

# Define the host and container directories for sharing
export HOST_DIR="/home/migbot/Dockerfile/docker_folder"
export CONTAINER_DIR="/home/shared"


# Check for container image
if [ -z "$CONTAINER_IMAGE" ]; then
    echo 'ERROR: The container image to run must be set with the --container or -c options'
    exit 1
fi

# Check if the image exists locally
if ! docker images -q "$CONTAINER_IMAGE" > /dev/null; then
    echo "ERROR: The container image $CONTAINER_IMAGE does not exist locally."
    exit 1
fi

# Check for display
DISPLAY_DEVICE=""

if [ -n "$DISPLAY" ]; then
    # Give docker root user X11 permissions
    sudo xhost +si:localuser:root
    
    # Enable SSH X11 forwarding inside container (https://stackoverflow.com/q/48235040)
    XAUTH=/tmp/.docker.xauth
    xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge -
    chmod 777 "$XAUTH"

    DISPLAY_DEVICE="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH"
fi

# Run the container
# For ROS communication Issue: https://github.com/eProsima/Fast-DDS/issues/1698
if docker ps -a -q -f name="$CONTAINER_NAME" | grep -q .; then
    if docker ps -q -f name="$CONTAINER_NAME" | grep -q .; then
        # If container is running, exec into it
        docker exec -ti "$CONTAINER_NAME" /bin/bash
    else
        # If container exists but is not running, start it
        echo "Starting existing docker container"
        docker start -ai "$CONTAINER_NAME"
    fi
else
    # If container does not exist, create and start it
    echo "Create docker"
    docker create -it \
        --name "$CONTAINER_NAME" \
        --network host \
        -v /dev/dri:/dev/dri \
        -v /dev/shm:/dev/shm \
        -v "$HOST_DIR":"$CONTAINER_DIR" \
        $DISPLAY_DEVICE \
        "$CONTAINER_IMAGE"
    docker start -ai "$CONTAINER_NAME"
fi


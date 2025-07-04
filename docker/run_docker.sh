#!/bin/bash

# Default options
DOCKER=tap-vins-img
NAME=tap_vins


XAUTH=$(pwd)/.docker.xauth

echo "Current DISPLAY: $DISPLAY"
echo "Preparing fresh Xauthority data..."

# Delete existing .docker.xauth if it exists
if [ -f "$XAUTH" ]; then
    echo "Removing existing Xauthority file: $XAUTH"
    rm -f "$XAUTH"
fi

# Get the xauth entries for the current display
xauth_list=$(xauth nlist $DISPLAY | sed -e 's/^..../ffff/')

if [ -n "$xauth_list" ]; then
    echo "$xauth_list" | xauth -f "$XAUTH" nmerge -
else
    echo "No xauth data found. Creating empty file."
    touch "$XAUTH"
fi

chmod a+r "$XAUTH"

echo "Xauthority setup complete at $XAUTH"

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

docker run -it \
    --gpus all \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --name=$NAME \
    ${DOCKER} \
    bash

echo "Done."
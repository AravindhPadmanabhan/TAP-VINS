XAUTH=$(pwd)/.docker.xauth

echo "\$DISPLAY: $DISPLAY"
echo "Preparing Xauthority data..."
# Get the correct DISPLAY (should be something like :10.0)
DISPLAY_NUM=$(echo $DISPLAY | cut -d: -f2)

# Extract the xauth entry for the forwarded display
xauth_list=$(xauth nlist $DISPLAY | sed -e 's/^..../ffff/')
# xauth_list=$(xauth nlist :1 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ -n "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."

if pgrep -x "z1_ctrl" > /dev/null; then
    echo "Another instance is already running."
else
    # Start the process
    cd src/arm/arm_z1/z1_controller/build/ && ./z1_ctrl >/dev/null 2>&1 &
    echo "controller has started"
    ps
fi
#!/bin/bash
# filepath: /home/cjh/zhujiang_ws/start/start2.sh

# Set password
password="123"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "=== Shutting down all services ==="
    
    # Stop Docker container
    if docker ps | grep -q "zhujiang"; then
        echo "Stopping Docker container..."
        docker stop zhujiang
    fi
    
    # Kill any remaining ROS processes
    pkill -f roslaunch 2>/dev/null || true
    pkill -f roscore 2>/dev/null || true
    
    echo "All services stopped. Goodbye!"
    exit 0
}

# Set trap to catch exit signals
trap cleanup EXIT INT TERM

# Allow X11 forwarding for Docker
echo "Setting up X11 forwarding for Docker..."
xhost +local:docker

echo "Starting Docker container in terminal..."

# Launch Docker container in a new terminator tab
terminator -e "docker run -it --rm \
  --privileged=true \
  --name zhujiang \
  --network=host \
  --ipc=host \
  --pid=host \
  -e DISPLAY=$DISPLAY \
  -e LANG=C.UTF-8 \
  -e LC_ALL=C.UTF-8 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v /home/$USER:/home/$USER \
  --group-add audio \
  --group-add video \
  zhujiang:v1 /bin/bash -c \"
    echo 'Docker container started successfully!'
    
    # Set up X11 in container
    export DISPLAY=$DISPLAY
    
    # Launch terminator with custom layout inside Docker
    terminator -l nav &
    sleep 3

    # Set USB permissions and launch LiDAR camera nodes
    echo 'Setting USB permissions and launching sensors...'
    terminator -e \\\"echo '123' | sudo -S chmod 666 /dev/ttyUSB0 /dev/ttyUSB1 && cd /home/cjh/nav_ws && source devel/setup.bash && roslaunch zj_nav zj_lidarcamera.launch\\\" --new-tab -T \\\"LiDAR_Camera\\\"
    sleep 3

    # Launch localization and aruco
    echo 'Launching localization and aruco...'
    terminator -e \\\"cd /home/cjh/nav_ws && source devel/setup.bash && roslaunch zj_nav zj_localize.launch\\\" --new-tab -T \\\"Localization\\\"
    sleep 3

    # Publish initial pose
    echo 'Publishing initial pose...'
    terminator -e \\\"cd /home/cjh/zhujiang_ws && source devel/setup.bash && rostopic pub /start_lio std_msgs/Bool true --once\\\" --new-tab -T \\\"Initialization\\\"
    sleep 15

    # Launch navigation
    echo 'Launching navigation...'
    terminator -e \\\"cd /home/cjh/nav_ws && source devel/setup.bash && roslaunch zj_nav zj_movebase.launch\\\" --new-tab -T \\\"Navigation\\\"
    
    echo 'Docker services initialized successfully!'
    echo 'Container will stop when parent script exits'
    
    # Keep container running
    tail -f /dev/null
\"" --new-tab -T "Docker_Container" &

# Wait for Docker container to be ready
echo "Waiting for Docker container to initialize..."
sleep 15

# Check if Docker container is running
while ! docker ps | grep -q "zhujiang"; do
    echo "Waiting for Docker container..."
    sleep 2
done

echo "Docker container is ready. Starting host services..."
sleep 5

# Configure CAN interface on host
echo "Configuring CAN interface on host..."
echo $password | sudo -S ip link set can0 up type can bitrate 500000

# Launch Tracer on host
echo "Launching Tracer base on host..."
terminator -e "cd /home/cjh/zhujiang_ws && source devel/setup.bash && roslaunch master_pkg start.launch" --new-tab -T "Master"

echo "All systems initialized successfully!"
echo "Docker container: zhujiang is running with navigation stack in 'Docker_Container' tab"
echo "Host: CAN interface configured and Tracer launched in 'Master' tab"
echo ""
echo "Press Ctrl+C to stop all services and exit"

# Keep script running until interrupted
while true; do
    sleep 1
    # Check if container is still running
    if ! docker ps | grep -q "zhujiang"; then
        echo "Docker container stopped unexpectedly. Exiting..."
        break
    fi
done

# Cleanup X11 permissions
# xhost -local:docker
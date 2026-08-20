#!/bin/bash

# SOURCE ROS ENVIRONMENT
# Ensure we're using system Python (not venv)
unset VIRTUAL_ENV
unset PYTHONPATH
export PATH="/usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin:$PATH"

source /opt/ros/humble/setup.bash
if [[ -f "/home/alessandro/int_sys_fp/install/setup.bash" ]]; then
    source /home/alessandro/int_sys_fp/install/setup.bash
else
    echo "⚠️  Local setup.bash not found, using only ROS2 environment"
fi
echo "🤖 === 3 TURTLEBOT3 CON GUI - SOLUZIONE FINALE ==="

# Test if required Python modules are available
echo "🔍 Testing Python environment..."
if ! python3 -c "import lxml" 2>/dev/null; then
    echo "❌ lxml module not found! Installing..."
    sudo apt install -y python3-lxml
    if ! python3 -c "import lxml" 2>/dev/null; then
        echo "❌ Failed to install lxml. Exiting."
        exit 1
    fi
fi
echo "✅ Python environment OK"

# Cleanup processi precedenti
pkill -f gazebo 2>/dev/null
sleep 2

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

# Set ROS2 to use simulation time from Gazebo
export ROS_DOMAIN_ID=0
ros2 param set /use_sim_time true 2>/dev/null || echo "Will set use_sim_time after nodes start"

echo "📋 Metodo: Launch singolo robot + spawn manuale degli altri 2"
echo "⏰ ROS2 configurato per usare simulation time di Gazebo"
echo ""

echo "🚀 Step 1: Lancio Gazebo (empty world)..."
# Launch empty world without robots
ros2 launch turtlebot3_gazebo empty_world.launch.py gui:=true &
LAUNCH_PID=$!
echo "⏱️  waiting 15 sec for Gazebo to load..."
sleep 15

# Check if Gazebo is running properly
if ! pgrep -f gzserver > /dev/null; then
    echo "❌ Gazebo server failed to start. Checking for errors..."
    echo "💡 This might be due to display issues or missing dependencies."
    exit 1
fi

echo "✅ Gazebo server is running"

echo "⏰ Setting global simulation time parameter..."
# Set the global use_sim_time parameter for all nodes
ros2 param set /use_sim_time true 2>/dev/null || echo "Global sim time will be set by individual nodes"
sleep 2

echo "✅ Primo robot già spawnato automaticamente da empty_world.launch.py in (0,0)"
echo "   Topics: /cmd_vel, /odom (namespace default)"
echo ""

echo "🚀 Step 2: Spawn secondo robot..."
echo "Debug: Spawning robot 2 at position (1.0, 0.0, 0.01) with namespace tb3_2"
ros2 run gazebo_ros spawn_entity.py \
    -entity turtlebot3_burger_2 \
    -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
    -x 1.0 -y 0.0 -z 0.01 \
    -robot_namespace tb3_2 &
SPAWN2_PID=$!

sleep 3

# Check if spawn was successful
if ! wait $SPAWN2_PID; then
    echo "⚠️  Second robot spawn may have failed, but continuing..."
    echo "Debug: Exit code for robot 2 spawn: $?"
else
    echo "✅ Robot 2 spawned successfully"
fi

echo "🚀 Step 3: Spawn terzo robot..."
echo "Debug: Spawning robot 3 at position (0.0, 1.0, 0.01) with namespace tb3_3"
ros2 run gazebo_ros spawn_entity.py \
    -entity turtlebot3_burger_3 \
    -file /opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
    -x 0.0 -y 1.0 -z 0.01 \
    -robot_namespace tb3_3 &
SPAWN3_PID=$!

sleep 3

# Check if spawn was successful
if ! wait $SPAWN3_PID; then
    echo "⚠️  Third robot spawn may have failed, but continuing..."
    echo "Debug: Exit code for robot 3 spawn: $?"
else
    echo "✅ Robot 3 spawned successfully"
fi

echo ""
echo "✅ 3 ACTIVE ROBOTS!"
echo ""
echo "📡 AVAILABLE TOPICS:"
echo "   Robot 1: /cmd_vel, /odom (namespace: default)"
echo "   Robot 2: /tb3_2/cmd_vel, /tb3_2/odom (namespace: tb3_2)"  
echo "   Robot 3: /tb3_3/cmd_vel, /tb3_3/odom (namespace: tb3_3)"
echo ""

echo "🚀 Step 4: UWB Sensor Emulator and noise configuration..."
echo ""
echo "🎛️  UWB NOISE CONFIGURATION:"
echo "Which type of noise you want to use?"
echo "1. Gaussian : (type 1)"
echo "2. Uniform  : (type 2)"
read -p "Enter choice (1 or 2): " NOISE_CHOICE

# Validate input
if [[ "$NOISE_CHOICE" != "1" && "$NOISE_CHOICE" != "2" ]]; then
    echo "⚠️  Invalid choice, defaulting to Gaussian (1)"
    NOISE_CHOICE=1
fi

echo "✅ Selected noise type: $NOISE_CHOICE"
echo ""

cd /home/alessandro/int_sys_fp/int_sys_fp

# Check if setup.bash exists before sourcing
if [[ -f "install/setup.bash" ]]; then
    source install/setup.bash
    echo "📦 int_sys_fp package sourced successfully"
else
    echo "⚠️  install/setup.bash not found. UWB emulator may not work properly."
fi

# Check if UWB emulator exists
if ros2 pkg list | grep -q int_sys_fp; then
    # Pass the choice to the UWB emulator via pipe and set sim time
    echo $NOISE_CHOICE | ros2 run int_sys_fp uwb_emulator --ros-args -p use_sim_time:=true &
    UWB_PID=$!
    echo "✅ UWB Emulator started with simulation time"
else
    echo "⚠️  int_sys_fp package not found. UWB emulator will not start."
    UWB_PID=""
fi

echo "� Step 5: Kalman Filter node startup..."
sleep 1

# Check if KF.py exists and is executable
KF_PATH="/home/alessandro/int_sys_fp/int_sys_fp/src/KF.py"
if [[ -f "$KF_PATH" ]]; then
    # Make sure it's executable
    chmod +x "$KF_PATH"
    
    # Start the Kalman Filter node with simulation time
    ros2 run int_sys_fp KF.py --ros-args -p use_sim_time:=true &
    KF_PID=$!
    echo "✅ Kalman Filter node started with simulation time (PID: $KF_PID)"
    sleep 2
else
    echo "⚠️  KF.py not found at $KF_PATH"
    echo "💡 Make sure the file exists and is in the correct location"
    KF_PID=""
fi

echo "�📊 Step 6: Plotjuggler startup for data visualization..."
sleep 2

ros2 run plotjuggler plotjuggler &
PLOT_PID=$!

sleep 2

echo ""
echo "🎯 === SISTEMA COMPLETO ATTIVO ==="
echo "🤖 3 TurtleBot3 con GUI Gazebo"
echo "📡 UWB Emulator attivo"
echo "🧠 Kalman Filter node attivo"
echo "📊 PlotJuggler per visualizzazione"
echo ""
echo "📈 TOPIC UWB DA MONITORARE:"
echo "   /uwb/robot_distances - Distanze tra robot"
echo "   /uwb/anchor_distances - Distanze da anchor"
echo ""
echo "� TOPIC KALMAN FILTER DA MONITORARE:"
echo "   /uwb/anchor_distances_filtered    "
echo "   /uwb/robot_distances_filtered    "
echo "   //covariance - Matrici di covarianza"
echo ""
echo "�💡 Press Ctrl+C to stop all"

# Cleanup function
cleanup() {
    echo "🧹 Shutting down system..."
    [[ -n "$KF_PID" ]] && kill $KF_PID 2>/dev/null
    [[ -n "$PLOT_PID" ]] && kill $PLOT_PID 2>/dev/null
    [[ -n "$RVIZ_PID" ]] && kill $RVIZ_PID 2>/dev/null
    [[ -n "$UWB_PID" ]] && kill $UWB_PID 2>/dev/null
    [[ -n "$SPAWN1_PID" ]] && kill $SPAWN1_PID 2>/dev/null
    [[ -n "$SPAWN2_PID" ]] && kill $SPAWN2_PID 2>/dev/null
    [[ -n "$SPAWN3_PID" ]] && kill $SPAWN3_PID 2>/dev/null
    [[ -n "$XVFB_PID" ]] && kill $XVFB_PID 2>/dev/null
    pkill -f gazebo 2>/dev/null
    echo "✅ All processes stopped"
}

trap cleanup EXIT

# Keep running
wait $LAUNCH_PID

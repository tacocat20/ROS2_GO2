#!/bin/bash

# Simple simulation launch script for obstacle avoidance
# Starts Gazebo + RViz + Obstacle Avoider Node

set -e

cd ~/ros2_ws
source install/setup.bash

echo "🤖 Starting Obstacle Avoidance Simulation..."
echo "   Gazebo + RViz + Avoider Node"
echo ""

# Get parameters (optional)
SAFE_DISTANCE="${1:-0.8}"
FORWARD_SPEED="${2:-0.3}"

echo "⚙️  Parameters:"
echo "   safe_distance: $SAFE_DISTANCE m"
echo "   forward_speed: $FORWARD_SPEED m/s"
echo ""

# Start Gazebo in background
echo "🌍 Starting Gazebo world..."
gazebo --verbose ~/ros2_ws/src/obstacle_avoidance/worlds/obstacle_world.world &
GAZEBO_PID=$!
sleep 5  # Wait for Gazebo to start

# Start RViz in background
echo "📊 Starting RViz visualization..."
rviz2 -d ~/ros2_ws/src/obstacle_avoidance/rviz/simulation.rviz &
RVIZ_PID=$!
sleep 2

# Start obstacle avoidance node (foreground so we can see output)
echo "🚗 Starting obstacle avoidance node..."
ros2 run obstacle_avoidance avoider --ros-args \
  -p safe_distance:=$SAFE_DISTANCE \
  -p forward_speed:=$FORWARD_SPEED \
  -p turn_speed:=1.5 \
  -p critical_distance:=0.4

# Cleanup on exit
trap "kill $GAZEBO_PID $RVIZ_PID 2>/dev/null; echo 'Simulation stopped'" EXIT

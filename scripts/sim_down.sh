#!/usr/bin/env bash
# Stop everything sim_up.sh started (guidebook 4.2).
tmux kill-session -t sim 2>/dev/null || true
pkill -f "gz sim" || true
pkill -f arducopter || true
pkill -f "ros2 launch drone_bringup" || true
pkill -f mavproxy || true
sleep 2
echo "simulation stopped"

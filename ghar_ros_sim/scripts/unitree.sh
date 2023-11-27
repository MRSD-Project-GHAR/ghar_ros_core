#!/bin/bash

# Kill previous session
tmux kill-session

# Create a new tmux session
session_name="ghar_unitree_sim_$(date +%s)"
tmux new-session -d -s $session_name

tmux set -g mouse on

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 2    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves

# Run the gazebo command in the first pane
tmux select-pane -t 0
tmux send-keys "roslaunch ghar_ros_unitree unitree_gazebo_rtab_nav.launch" Enter

# Run the control in the second pane
tmux select-pane -t 1
tmux send-keys "sleep 10 && rosrun unitree_guide junior_ctrl" Enter
tmux send-keys "sleep 13 && 2 && sleep 3 && 5" Enter

# Run the gridmap filter script in the third pane
tmux select-pane -t 2
tmux send-keys "sleep 10 && roslaunch ghar_ros_unitree map_filter_unitree.launch" Enter

# Run the planner script in the third pane
tmux select-pane -t 3
tmux send-keys "sleep 10 && roslaunch ghar_ros_unitree planner_move_base_unitree.launch" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
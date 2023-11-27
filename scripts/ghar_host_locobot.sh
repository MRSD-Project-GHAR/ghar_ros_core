#!/bin/bash

gnome-termial

# Kill previous session
tmux kill-session

# Create a new tmux session
session_name="ghar_locobot_host_$(date +%s)"
tmux new-session -d -s $session_name

tmux set -g mouse on

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves
# tmux selectp -t 2    # go back to the first pane
# tmux splitw -h -p 50 # split it into two halves

# Run the camera command in the first pane
tmux select-pane -t 0
tmux send-keys "roslaunch rand_grid_map_gen map_publisher.launch" Enter

# Run the RTAB in the second pane
tmux select-pane -t 1
tmux send-keys 'roslaunch rand_grid_map_gen map_filter.launch robot_name:=locobot' Enter

# Run the control script in the third pane
tmux select-pane -t 2
tmux send-keys "sleep 10 && roslaunch mbf_rrts_planner planner_move_base.launch robot_name:=locobot # Wait for the move_base server to start on the robot if you want to do local planning" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
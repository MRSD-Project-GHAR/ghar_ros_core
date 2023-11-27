#!/bin/bash

gnome-termial

# Kill previous session
tmux kill-session

# Create a new tmux session
session_name="ghar_locobot_$(date +%s)"
tmux new-session -d -s $session_name

tmux set -g mouse on

# Split the window into three panes
tmux selectp -t 0    # select the first (0) pane
tmux splitw -v -p 50 # split it into two halves
tmux selectp -t 0    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves
tmux selectp -t 2    # go back to the first pane
tmux splitw -h -p 50 # split it into two halves

# Run the camera command in the first pane
tmux select-pane -t 0
tmux send-keys "roslaunch realsense2_camera rs_camera.launch enable_infra1:=true enable_infra2:=true" Enter

# Run the RTAB in the second pane
tmux select-pane -t 1
tmux send-keys 'sleep 10 && cp rtabmap_nsh_4th_19th_nov_2_loops_forward_1_loop_backward_only_stereo_corrected_world.db .ros/rtabmap_nsh_4th.db && roslaunch ghar_ros_common rtabmap_ghar_stereo.launch' Enter

# Run the control script in the third pane
tmux select-pane -t 2
tmux send-keys "roslaunch ghar_ros_locobot xslocobot_nav_empty.launch robot_model:=locobot_wx200 use_lidar:=false" Enter

# Run the keyop script for backup in the fourth pane
tmux select-pane -t 3
tmux send-keys "sleep 5 && roslaunch kobuki_keyop keyop.launch __ns:=locobot" Enter

# Attach to the tmux session
tmux -2 attach-session -t $session_name
# ghar_ros_core

On the locobot - 

roslaunch realsense2_camera rs_camera.launch enable_infra1:=true enable_infra2:=true

roslaunch ghar_ros_common rtabmap_ghar_stereo.launch

roslaunch ghar_ros_locobot xslocobot_nav_empty.launch robot_model:=locobot_wx200 use_lidar:=false

On the remote computer
roslaunch rand_grid_map_gen map_publisher.launch

roslaunch rand_grid_map_gen map_filter.launch robot_name:=locobot

roslaunch mbf_rrts_planner planner_move_base.launch robot_name:=locobot

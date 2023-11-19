# ghar_ros_core

## On the locobot
On the robot:
~~~
roslaunch realsense2_camera rs_camera.launch enable_infra1:=true enable_infra2:=true
cp ~/rtabmap_nsh_4th_19th_nov_2_loops_forward_1_loop_backward_only_stereo_corrected_world.db ~/.ros/rtabmap_nsh_4th.db && roslaunch ghar_ros_common rtabmap_ghar_stereo.launch
roslaunch ghar_ros_locobot xslocobot_nav_empty.launch robot_model:=locobot_wx200 use_lidar:=false
~~~

On the remote computer
~~~
roslaunch rand_grid_map_gen map_publisher.launch
roslaunch rand_grid_map_gen map_filter.launch robot_name:=locobot
roslaunch mbf_rrts_planner planner_move_base.launch robot_name:=locobot # Wait for the move_bse server to start on the robot if you want to do local planning
~~~

## On the A1
On the robot:
~~~
roslaunch realsense2_camera rs_camera.launch enable_infra1:=true enable_infra2:=true
cp ~/rtabmap_nsh_4th_19th_nov_2_loops_forward_1_loop_backward_only_stereo_corrected_world.db ~/.ros/rtabmap_nsh_4th.db && roslaunch ghar_ros_common rtabmap_ghar_stereo.launch
roslaunch ghar_ros_go1 move_base_go1.launch 
~~~

On the remote computer
~~~
roslaunch rand_grid_map_gen map_publisher.launch
roslaunch rand_grid_map_gen map_filter.launch robot_name:=go1
roslaunch mbf_rrts_planner planner_move_base.launch robot_name:=go1 # Wait for the move_bse server to start on the robot if you want to do local planning
~~~

# Commands for Turtlebot3 simulation

## Launch the simulation 
```
export TURTLEBOT3_MODEL=waffle
roslaunch ghar_ros_turtlebot3 turtlesim3_waffle_demo_rtab_nav.launch
```

## Launch the map filter node
```
roslaunch rand_grid_map_gen map_filter.launch robot_name:=turtlebot3 grid_map_topic:=/elevation_mapping/elevation_map 
```
## Launch the RRTS node
```
roslaunch ghar_ros_turtlebot3 planner_move_base_turtlebot3.launch
```

<!-- ## Get the points from RVIZ using publish point button -->


# Commands for Unitree Go1 simulation

## Launch the simulation 
```
roslaunch ghar_ros_unitree unitree_gazebo_rtab_nav.launch
```

## Launch the control node
```
rosrun unitree_guide junior_ctrl
```

## Launch the map filter node
```
roslaunch rand_grid_map_gen map_filter.launch robot_name:=unitree grid_map_topic:=/elevation_mapping/elevation_map 
```
## Launch the RRTS node
```
roslaunch ghar_ros_unitree planner_move_base_unitree.launch
```

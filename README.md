# Step 1: Run roscore
~$ roscore

# Step 2: Create workspace

~$ mkdir -p catkin_ws/src

~$ cd catkin_ws/src

~$ catkin_init_workspace

# Step 3: Initialize package 
Copy [grid_maker] into [catkin_ws/src]

~$ cd catkin_ws

~$ catkin_make

~$ source devel/setup.bash

# Step 4: Modify your code
... ...

# Step 5: Run your package
~$ cd catkin_ws | catkin_make

~$ rosrun [package_name] [node_name] # rosrun grid_maker drawGrip

# Step 6: Run rviz
~$ rosrun rviz rviz

Setting:

## Gloabal Option

Fixd Frame := odom

## Grid
Plane Cell Count := 100 	# Max Grid Number
Cell Size := 0.5 			# (m) 

## Button [Add]
Select [rviz/Axes]
Length := 0.5				# (m)
Radius := 0.03				# (m)

## Button [Add]
Select [rviz/MarkerArray]
Marker Topic := visualization_maker_array
Queue Size := 500


[Attention]
Text format: x,y (unit: m)
4,5
6,-9
2.3,9.0
-2.4,0.01

# SemaFORR Package
This package contains the core semaforr navigation system

How to use:
* `ros2 run semaforr (node)`

Available Nodes:
* `semaforr_node`: This node runs the main.cpp file used to launch the semaforr navigation system. It requires 6 parameters.

To Run (edit paths if necessary):
* `ros2 run semaforr semaforr_node --ros-args -p semaforr_path:="/root/semaforr_ros2/src/semaforr" -p target_set:="/root/semaforr_ros2/src/semaforr/config/stage_tutorial/target.conf" -p map_config:="/root/semaforr_ros2/src/semaforr/config/stage_tutorial/stage_tutorialS.xml" -p map_dimensions:="/root/semaforr_ros2/src/semaforr/config/stage_tutorial/dimensions.conf" -p advisors:="/root/semaforr_ros2/src/semaforr/config/advisors.conf" -p params:="/root/semaforr_ros2/src/semaforr/config/params.conf"`
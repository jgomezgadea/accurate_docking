# accurate_docking

This package contains a node (**accurate_docking**) that automates the accurate dock of the robot to a point.


## 1 accurate_docking_node

Test program to perform a docking to any frame with the maximum accuracy.
It combines a set of movement primitives [*dock, dock, rotate, move forward, move backward*] to reach the desired target goal, measure the error and get back to the initial position.
It also saves the errors comitted on every iteration. These errors can be exported to csv.
Component based on Robotnik [RComponet](https://github.com/RobotnikAutomation/rcomponent).

### 1.1 Parameters

* **~dock_action_server (string, default: pp_docker)**
   Robotnik's docker action server used
* **~move_action_server (string, default: move)**
   Robotnik's move action server used
* **~pregoal_frame (string, default: laser_test_pregoal_footprint)**
   Target frame to perform a pregoal approach. This frame can be the same one than goal frame
* **~goal_frame (string, default: laser_test_goal_footprint)**
   Target frame
* **~base_frame (string, default: robot_base_footprint)**
   Robot's base frame
* **~pregoal_offset_1([], default: [-0.5, 0, 0])**
   Offset (x, y, theta) applied to the first dock action 
* **~pregoal_offset_1([], default: [-0.5, 0, 0])**
   Offset (x, y, theta) applied to the first dock action
* **~step_back_distance([]double, default: 1.3)**
   Distance to move back the robot after reaching the target goal and measuring the error.
* **~consecutive_iterations(int, default: 1])**
   Number of consecutive iterations performed
   
   
### 1.2 Subscribed Topics

None

### 1.3 Published Topics

See RComponent based topics.

### 1.4 Services

* **~start (std_srvs/Trigger)**
  Starts the process.
* **~stop (std_srvs/Trigger)**
  Stops the process after finishing the current iteration
  * **~save_results (std_srvs/Empty)**
  Saves the error results into a file.csv in the package/data folder.

### 1.5 Services Called

None

### 1.6 Action server

None

### 1.7 Action clients called

*  **dock_action_server(robotnik_navigation_msgs/DockAction)**
  Action that performs a dock into any frame
*  **move_action_server(robotnik_navigation_msgs/MoveAction)**
  Action that performs a relative movement in either x, y or theta.

### 1.8 Required tf Transforms

* base_frame → goal_frame
  Transform between base and goal

### 1.9 Provided tf Transforms

None

### 1.10 Bringup

Running the node that performs the tests
```
roslaunch accurate_docking accurate_docking.launch
```

Running the detectors, docker, move and transforms to perform the tests
```
roslaunch accurate_docking detector_and_docker.launch
```


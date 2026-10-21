# Mission-Level-Route-Planner

## Building Mission Level Route Planner Package 

In the root of the workspace

```
cd /Mission-Level-Route-Planner/ros2_ws/
```

Run

```
colcon build
```

Now the packages are built cd into```ros2_ws``` and source the setup file 

```
source install/local_setup.bash
```

Now the package are built, you can run the node. Use the package with ros2 sourced

```
ros2 run mission_level_route_planner mission_level_route_planner_node
```

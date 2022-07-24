
# HOW TO run the project

## Node by node

### Gazebo/RViz and nav stack

shell 1:

```
rosclean purge -y
roslaunch robocluedo_robot_2 demo_gazebo.launch

```

shell 2:

```
roslaunch robocluedo_robot_2 nav_stack.launch
```

### robocluedo rosplan interface

shell 3:

```
rosrun erl2 simulation &
roslaunch robocluedo_rosplan_interface run_project.launch
```

### robocluedo movement controller

shell 4:

```
roslaunch robocluedo_movement_controller run_movement_controller.launch
```

### robocluedo mission manager

shell 5:

```
roslaunch robocluedo_mission_manager run_units.launch
```

shell 6:

```
rosrun robocluedo_mission_manager planning_unit
```

## launch files

### Gazebo/RViz and nav stack

```

```

### robocluedo rosplan interface

```

```

### robocluedo movement controller

```

```

### robocluedo mission manager

```

```

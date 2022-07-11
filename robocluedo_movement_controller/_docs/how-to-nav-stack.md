# HOW TO -- navigation stack CRASH COURSE

here are some quick informations about how to set up and use the **ROS Navigation Stack**.

## install the Navigation Stack

### Slam - GMapping

In order to run the project, you need the two packages you can find [here](https://github.com/CarmineD8/slam_gmapping.git). SLAM and GMapping are tools for managing the movement of a robot with noisy odometry: their purpose is to correct odometry in a way that the robot can get its position as precisely as possible. 

Copy the packages into the workspace you prefer. **use branch : _noetic_**


```bash
git clone https://github.com/CarmineD8/slam_gmapping.git -b noetic
```

Also these packages are required. Please install them. 

```bash
sudo apt-get install ros-noetic-openslam-gmapping
sudo apt-get install ros-noetic-navigation
```

### MoveBase

MoveBase is a motion planner: given a goal, it can retrieve a path from the actual position to the desired one, recomputing the path depending on the informations gathered by sensors in conjunction with Slam-GMapping. 

Simply install it:

```bash
sudo apt-get install ros-noetic-move-base
```

## A quick example with Gazebo and RViz

see [this project on GitHub](https://github.com/programmatoroSeduto/RT1_assignment_2)

```bash
git clone https://github.com/programmatoroSeduto/RT1_assignment_2.git -b main ./test_nav_stack
```

in particular see [how to setup the project](https://github.com/programmatoroSeduto/RT1_assignment_2#how-to-set-up-the-project)

and see [how to build the project](https://github.com/programmatoroSeduto/RT1_assignment_2#build-the-project)

## Interfaces 

**services**:

```
/move_base/NavfnROS/make_plan
/move_base/TrajectoryPlannerROS/set_parameters
/move_base/clear_costmaps
/move_base/global_costmap/inflation_layer/set_parameters
/move_base/global_costmap/obstacle_layer/set_parameters
/move_base/global_costmap/set_parameters
/move_base/global_costmap/static_layer/set_parameters
/move_base/local_costmap/inflation_layer/set_parameters
/move_base/local_costmap/obstacle_layer/set_parameters
/move_base/local_costmap/set_parameters
/move_base/make_plan
/move_base/set_parameters
```

**topics:**

```
/move_base/NavfnROS/plan
/move_base/TrajectoryPlannerROS/cost_cloud
/move_base/TrajectoryPlannerROS/global_plan
/move_base/TrajectoryPlannerROS/local_plan
/move_base/TrajectoryPlannerROS/parameter_descriptions
/move_base/TrajectoryPlannerROS/parameter_updates
/move_base/cancel
/move_base/current_goal
/move_base/feedback
/move_base/global_costmap/costmap
/move_base/global_costmap/costmap_updates
/move_base/global_costmap/footprint
/move_base/global_costmap/inflation_layer/parameter_descriptions
/move_base/global_costmap/inflation_layer/parameter_updates
/move_base/global_costmap/obstacle_layer/clearing_endpoints
/move_base/global_costmap/obstacle_layer/parameter_descriptions
/move_base/global_costmap/obstacle_layer/parameter_updates
/move_base/global_costmap/parameter_descriptions
/move_base/global_costmap/parameter_updates
/move_base/global_costmap/static_layer/parameter_descriptions
/move_base/global_costmap/static_layer/parameter_updates
/move_base/goal
/move_base/local_costmap/costmap
/move_base/local_costmap/costmap_updates
/move_base/local_costmap/footprint
/move_base/local_costmap/inflation_layer/parameter_descriptions
/move_base/local_costmap/inflation_layer/parameter_updates
/move_base/local_costmap/obstacle_layer/clearing_endpoints
/move_base/local_costmap/obstacle_layer/parameter_descriptions
/move_base/local_costmap/obstacle_layer/parameter_updates
/move_base/local_costmap/parameter_descriptions
/move_base/local_costmap/parameter_updates
/move_base/parameter_descriptions
/move_base/parameter_updates
/move_base/result
/move_base/status
/move_base_simple/goal
```

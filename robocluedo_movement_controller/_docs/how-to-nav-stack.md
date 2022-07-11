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

## Names and Types of the Interfaces of MoveBase

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

## the action movebase

movebase is meant to behave as an action:

- you can send a 2D position to reach, topic `/move_base/goal` of type `move_base_msgs/MoveBaseAction`
- the fullfillment of the goal could require a lot of time
- finally, the move-base node will send a result message with the success flag set

for the complete list of interfaces opened by move-base, see [move_base_msgs package](http://docs.ros.org/en/fuerte/api/move_base_msgs/html/index-msg.html)

also [the official wiki](http://wiki.ros.org/move_base) is a good reference (very seldom, sometimes it happens...)

### A note about the action names

- *MoveBase.action* -- the name of the action, file with 3 fields

this message is split into 3 different messages:

- *MoveBaseActionGoal*
- *MoveBaseActionResult*
- *MoveBaseActionFeedback*

but *what you need are these names:* 

- *MoveBaseGoal*
- *MoveBaseResult*
- *MoveBaseFeedback*

note that in C++ you can refer to them in this way:

```c++
// include of the action
#include "move_base_msgs/MoveBaseAction.h"
// referring to the file MoveBase.action

// the goal will be
move_base_msgs::MoveBaseGoal goal;

// the feedback will be
move_base_msgs::MoveBaseFeedback feedback;

// and finally, the result will be
move_base_msgs::MoveBaseResult result;
```

### move_base_msgs/MoveBaseGoal

```
geometry_msgs/PoseStamped target_pose
```

### move_base_msgs/MoveBaseFeedback

```
geometry_msgs/PoseStamped base_position
```

### move_base_msgs/MoveBaseResult

**empty means that** the framework will sendo only a simple signal. 

```
# ... empty
```

## other interfaces

### geometry_msgs/PoseStamped

a pose with the recording time and *the frame*

```
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose
```

**remember to give a frame for the pose!** if you don't know what to take as pose frame, use this:

```python
msg.goal.target_pose.header.frame_id = "map"
```

### geometry_msgs/Pose

a pose is made up of

- a position
- and a orientation *as quaternion*

```
# A representation of pose in free space, composed of postion and orientation. 
Point position
Quaternion orientation
```

**Gazebo spatial convention**

- horizontal plane : **x** (red) and **y** (green)
- vertical axis : **z** (blue)

**Rotation ROS convention**

see the answer under [this post](https://answers.ros.org/question/58863/incorrect-rollpitch-yaw-values-using-getrpy/)

- *x* -- roll
- *y* -- pitch
- *z* -- yaw

### geometry_msgs/Quaternion

representation of a rotation using 4 parameters. 

```
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
```

**C++ how to convert quaternion to roll-pitch-yaw**

see [this simple example on GitHub Gist](https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac)

```c++
// tf contains some tool to deal with the most common geonetry for the Geometrical Unit
#include <tf/tf.h>

double roll, pitch, yaw;

tf::Quaternion q( x, y, z, w )
tf::Matrix3x3 m(q);
m.getRPY(roll, pitch, yaw);
```

### geometry_msgs/Point

```
# This contains the position of a point in free space
float64 x
float64 y
float64 z
```

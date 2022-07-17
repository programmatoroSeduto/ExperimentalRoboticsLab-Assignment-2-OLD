# Practical notes about MoveIt Setup Assistant

> 
> using the *latest version* (07/2022) 
> 

## what Setup Assistant really does

setup assistant has the advantage to automatically set up the controllers and to give a control structure enabling other nodes to drive the robot, which is a very boring task. 

*the dark side* of setup assitant is that, if you want to integrate other control frameworks such as move_base and GMapping, it doesn't work properly. moreover, there are other problems with the code generated, which is not so good typically. 

## Setup Assistant Help

just for reference:

```bash
# rosrun moveit_setup_assistant moveit_setup_assistant -h
Allowed options:
  -h [ --help ]           Show help message
  -g [ --debug ]          Run in debug/test mode
  -u [ --urdf_path ] arg  Optional, path to URDF file in ROS package
  -c [ --config_pkg ] arg Optional, pass in existing config package to load
```

## Gazebo Configuration

th the URDF file `robocluedo.urdf`

### ft_sensor

```xml
    <gazebo>
        <plugin filename="libgazebo_ros_ft_sensor.so" name="ft_sensor">
            <updateRate>100.0</updateRate>
            <topicName>ft_sensor_topic</topicName>
            <jointName>arm_joint_02</jointName>
        </plugin>
    </gazebo>
```

### differential drive controller

```
    <gazebo>
        <plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive_controller">
            <legacyMode>true</legacyMode>
            <alwaysOn>true</alwaysOn>
            <updateRate>20</updateRate>
            <leftJoint>joint_left_wheel</leftJoint>
            <rightJoint>joint_right_wheel</rightJoint>
            <wheelSeparation>0.3</wheelSeparation>
            <wheelDiameter>0.2</wheelDiameter>
            <torque>0.1</torque>
            <commandTopic>cmd_vel</commandTopic>
            <odometryTopic>odom</odometryTopic>
            <odometryFrame>odom</odometryFrame>
            <robotBaseFrame>base_link</robotBaseFrame>
            <rosDebugLevel>na</rosDebugLevel>
            <publishWheelTF>true</publishWheelTF>
            <publishWheelJointState>true</publishWheelJointState>
            <wheelAcceleration>0</wheelAcceleration>
            <wheelTorque>5</wheelTorque>
            <odometrySource>1</odometrySource>
            <publishTf>1</publishTf>
        </plugin>
    </gazebo>
```

### hokuyo laser range finder

```
  <!-- hokuyo -->
  <gazebo reference="base_link">
    <sensor name="head_hokuyo_sensor" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
      <update_rate>40</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-1.570796</min_angle>
            <max_angle>1.570796</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <!-- Noise parameters based on published spec for Hokuyo laser
               achieving "+-30mm" accuracy at range < 10m.  A mean of 0.0m and
               stddev of 0.01m will put 99.7% of samples within 0.03m of the true
               reading. -->
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="hokuyo_node" filename="libgazebo_ros_laser.so">
        <topicName>/scan</topicName>
        <frameName>base_link</frameName>
      </plugin>
    </sensor>
  </gazebo>
```

### gazebo ros control

```
     <gazebo>
        <plugin filename="libgazebo_ros_control.so" name="gazebo_ros_control">
            <robotNamespace>m2wr</robotNamespace>
            <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
            <legacyModeNS>true</legacyModeNS>
        </plugin>
    </gazebo>
    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
            <robotNamespace>/</robotNamespace>
        </plugin>
    </gazebo>
```

## first setps -- my setup

start the setup assistant: (sadly, it doesn't take relative paths)

```bash
roscore &
rosrun moveit_setup_assistant moveit_setup_assistant --urdf_path /root/ros_ws/src/ExperimentalRoboticsLab-Assignment-2/robocluedo_movement_controller/urdf/robocluedo.urdf
```

configuration:

- package from URDF file : `$(fin robocluedo_movement_controller)/urdf/robocluedo.urdf`
- *sef collision* : all the checkboxes checked (no collision detection)
- *no virtual joints*
- *groups* : 
	- `arm_group` with kinematic solver *cached_ik_kinematics_plugin/CachedKDLKinematicsPlugin*, group default planner *RRT*, chain from `arm_base_link` to `cluedo_link`
- *robot poses*:
	- `init` with all the joints set to zero
- *no end effector*
- *no passive joints*
- controllers:
	- (auto-generated) `arm_group_controller` for the kinematic chain `arm_group` of type `effort_controllers/JointTrajectoryController`
- *skip simulation*
- *skip 3D perception*
- ...name and email...
- **package name** : `robocluedo_robot`

now we have a package to work on. Remember to call `catkin_make` on the workspace. 

## FIRST (funny?) ISSUE -- robot explosion

try to open Gazebo now with `roslaunch robocluedo_robot demo_gazebo.launch` or also `roslaunch robocluedo_robot gazebo.launch` ... the robot will *blow up*! Funny, but ... definitely this is far from the desired behaviour. 

- [this post](https://github.com/Kinovarobotics/kinova-ros/issues/316) shows a similar problem ... just a source of inspiration, since the project are deeply different

this effect is due to a bad configuration of the parameters of the controllers. Go inside the file `robocluedo_robot/config/ros_controllers.yaml` and set these parameters (for each PID controller, we delete the derivative part and the constant part, leaving the proportional part only):

```yaml
arm_group_controller:
  type: effort_controllers/JointTrajectoryController
  joints:
    - arm_joint_01
    - arm_joint_02
    - arm_joint_03
    - arm_joint_04
  gains:
    arm_joint_01:
      p: 100
      d: 0
      i: 0
      i_clamp: 0
    arm_joint_02:
      p: 100
      d: 0
      i: 0
      i_clamp: 0
    arm_joint_03:
      p: 100
      d: 0
      i: 0
      i_clamp: 0
    arm_joint_04:
      p: 100
      d: 0
      i: 0
      i_clamp: 0
```

now the robot will oscillate a little, but at least it no longer explodes. so far, so good. 

## LASER SCAN

in order to make move_base and SLAM/GMapping work, we need to check if the laser sensor works as expected and it publishes on the topic `/scan`. we can use RViz for this purpose.

```bash
roslaunch robocluedo_robot demo_gazebo.launch
```

well, try to add in RViz the laser scan ... and ... TATTARATTAAAAAA! Not working!

try also to visualize the robot model, and (surprise surprise) : not working again! wonderful ...

### RViz ISSUE -- no transform from [x] to [base_link]

the error in the laser sensor `frame [laser] doesn't exist` is just an effect of the error on the geometrical representation of the robot. 

- see [this post](https://answers.ros.org/question/9365/no-transform-from-anything-to-base_link/), the error is almost identical to the mine, *even if it is a very old post* (from 2011 !!!)

the post is not so useful, but it suggested me an idea: since the error stands in how RViz is launched (as far as I can notice, in a wrong way for some reason), it could be a good idea to launch first Gazebo, and then RViz. two shells:

```bash
# SHELL 1 -- launch Gazebo
roslaunch robocluedo_robot gazebo.launch

```

```bash
# SHELL 2 -- launch RViz
roslaunch robocluedo_robot demo.launch

```

and ... TADAAAAA It seems to work now. 

the weird thing is that, in the launch file, *the execution order is exactly the one I used right now*. Here's the generated file `demo_gazebo.launch`:

```xml
<include file="$(dirname)/gazebo.launch" >
	<arg name="paused" value="$(arg paused)"/>
	<arg name="gazebo_gui" value="$(arg gazebo_gui)"/>
</include>

<include file="$(dirname)/demo.launch" pass_all_args="true">
	<arg name="load_robot_description" value="false" />
	<arg name="moveit_controller_manager" value="ros_control" />
</include>
```

## GMapping and move_base

a first attempt. first of all, let's take into account that

- the laser scan is published on the topic `/scan`
- the *differential drive* is a Gazebo plugin, not correlated with MoveIt: `/cmd_vel` as input, and `/odom` as output

### system setup

the folder `/robocluedo_robot/params` contains 5 files for move_base and SLAM/GMapping.

- global frame ID : *map*
- the laser scan is published on the topic `/scan`
- root link : *base_link*

The idea is this : first, launch the simulation environment as showed before; and finally, launch the navigation stack with another launch file `robocluedo_robot/launch/run_nav_stack.launch`

**file base_local_planner_params.yaml**

- global frame ID : *map*

```yaml
TrajectoryPlannerROS:

# Robot Configuration Parameters
  max_vel_x: 0.8
  min_vel_x: 0.4

  max_vel_theta:  1.5
  min_vel_theta:  -1.5
  min_in_place_vel_theta: -1.0
  
  acc_lim_x: 0.5
  acc_lim_theta: 1.0

# Goal Tolerance Parameters
  yaw_goal_tolerance: 3.14
  xy_goal_tolerance: 0.3

# Forward Simulation Parameters
  sim_time: 2.0
  vx_samples: 10
  vtheta_samples: 40

# Trajectory Scoring Parameters
  meter_scoring: true
  pdist_scale: 5.0
  gdist_scale: 0.0
  occdist_scale: 5.0
  heading_lookahead: 5.0
  dwa: false
  global_frame_id: map

# Oscillation Prevention Parameters
  oscillation_reset_dist: 0.05

# Differential-drive robot configuration
  holonomic_robot: false
  max_vel_y: 0.0
  min_vel_y: 0.0
  acc_lim_y: 0.0
  vy_samples: 0

```

**file costmap_common_params.yaml**

- the laser scan is published on the topic `/scan`

```yaml
max_obstacle_height: 0.60  
obstacle_range: 3.0
raytrace_range: 3.0
robot_radius: 0.15

# voxel map configuration; z-voxels 0 are filled by bumpers and 1 by laser scan (kinect)
map_type: voxel
origin_z: 0.0
z_resolution: 0.2
z_voxels: 2
publish_voxel_map: false

observation_sources: scan

scan: {data_type: LaserScan, topic: scan, marking: true, clearing: true, min_obstacle_height: 0.05, max_obstacle_height: 0.6}

```

**file global_costmap_params.yaml**

- global frame ID : *map*
- root link : *base_link*

```yaml
global_costmap:
   global_frame: map
   robot_base_frame: base_link
   update_frequency: 1.0
   publish_frequency: 0.5
   static_map: true
   transform_tolerance: 1.0
   inflation_layer:
        inflation_radius: 1.0
        cost_scaling_factor: 20

```

**file local_costmap_params.yaml**

- global frame ID : *map*
- root link : *base_link*

```yaml
local_costmap:
   global_frame: map
   robot_base_frame: base_link
   update_frequency: 5.0
   publish_frequency: 2.0
   static_map: false
   rolling_window: true
   width: 4.0
   height: 4.0
   resolution: 0.05
   transform_tolerance: 1.0
   inflation_layer:
        inflation_radius: 1.0
        cost_scaling_factor: 20

```

**file move_base_params.yaml**

```yaml
shutdown_costmaps: false

controller_frequency: 5.0
controller_patience: 5.0

planner_frequency: 5.0
planner_patience: 5.0

oscillation_timeout: 10.0
oscillation_distance: 0.05

```

**robocluedo_robot/launch/nav_stack.launch**:

```xml
<?xml version="1.0"?>

<launch>
	
	<param name="use_sim_time" value="true"/>
	
	<!-- SLAM/GMapping -->
	<node pkg="gmapping" type="slam_gmapping" name="slam_gmapping">
		<param name="base_frame" value="base_link"/>
		<param name="map_update_interval" value="5.0"/>
		<param name="maxUrange" value="16.0"/>
		<param name="sigma" value="0.05"/>
		<param name="kernelSize" value="1"/>
		<param name="lstep" value="0.05"/>
		<param name="astep" value="0.03"/>
		<param name="iterations" value="3"/>
		<param name="lsigma" value="0.075"/>
		<param name="ogain" value="3.0"/>
		<param name="lskip" value="0"/>
		<param name="srr" value="0.1"/>
		<param name="srt" value="0.1"/>
		<param name="str" value="0.1"/>
		<param name="stt" value="0.1"/>
		<param name="linearUpdate" value="1.0"/>
		<param name="angularUpdate" value="0.2"/>
		<param name="temporalUpdate" value="3.0"/>
		<param name="resampleThreshold" value="0.5"/>
		<param name="particles" value="20"/>
		<param name="xmin" value="-50.0"/>
		<param name="ymin" value="-50.0"/>
		<param name="xmax" value="50.0"/>
		<param name="ymax" value="50.0"/>
		<param name="delta" value="0.05"/>
		<param name="llsamplerange" value="0.01"/>
		<param name="llsamplestep" value="0.01"/>
		<param name="lasamplerange" value="0.005"/>
		<param name="lasamplestep" value="0.005"/>
		<param name="transform_publish_period" value="0.0005"/>
	</node>
	
	<!-- move base -->
	<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
		<rosparam file="$(find robocluedo_robot)/param/costmap_common_params.yaml" command="load" ns="global_costmap"/>
		<rosparam file="$(find robocluedo_robot)/param/costmap_common_params.yaml" command="load" ns="local_costmap"/>
		<rosparam file="$(find robocluedo_robot)/param/local_costmap_params.yaml" command="load"/>
		<rosparam file="$(find robocluedo_robot)/param/global_costmap_params.yaml" command="load"/>
		<rosparam file="$(find robocluedo_robot)/param/base_local_planner_params.yaml" command="load"/>
		<rosparam file="$(find robocluedo_robot)/param/move_base_params.yaml" command="load"/>
	</node>
	
</launch>
```

### running the system

**for the testing, i'm using 3 shells**:

```bash
# SHELL 1 -- launch Gazebo
roslaunch robocluedo_robot gazebo.launch world_file_path:=square_room.world

```

```bash
# SHELL 2 -- launch RViz
roslaunch robocluedo_robot demo.launch rviz_config_file:=sim_nav_stack.rviz

```

```bash
# SHELL 3 -- launch the navigation stack
roslaunch robocluedo_robot nav_stack.launch

```

### topics checkings

**when something goes wrong, move_base doesn't open up all the needed topics**, in particular the cost maps aren't issues. Hence, just check the topics and the subscriptions to understand if the system is working fine. 

in particular the third command will generate loads of warnings because of a well known issue under the hood. But you can skip them.

**list of topics** after launched the three components:

```
# rostopic list
/arm_group_controller/command
/arm_group_controller/follow_joint_trajectory/cancel
/arm_group_controller/follow_joint_trajectory/feedback
/arm_group_controller/follow_joint_trajectory/goal
/arm_group_controller/follow_joint_trajectory/result
/arm_group_controller/follow_joint_trajectory/status
/arm_group_controller/gains/arm_joint_01/parameter_descriptions
/arm_group_controller/gains/arm_joint_01/parameter_updates
/arm_group_controller/gains/arm_joint_02/parameter_descriptions
/arm_group_controller/gains/arm_joint_02/parameter_updates
/arm_group_controller/gains/arm_joint_03/parameter_descriptions
/arm_group_controller/gains/arm_joint_03/parameter_updates
/arm_group_controller/gains/arm_joint_04/parameter_descriptions
/arm_group_controller/gains/arm_joint_04/parameter_updates
/arm_group_controller/state
/attached_collision_object
/clock
/cmd_vel
/collision_object
/execute_trajectory/cancel
/execute_trajectory/feedback
/execute_trajectory/goal
/execute_trajectory/result
/execute_trajectory/status
/ft_sensor_topic
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/joint_states
/map
/map_metadata
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
/move_base/recovery_status
/move_base/result
/move_base/status
/move_base_simple/goal
/move_group/cancel
/move_group/display_contacts
/move_group/display_cost_sources
/move_group/display_grasp_markers
/move_group/display_planned_path
/move_group/fake_controller_joint_states
/move_group/feedback
/move_group/goal
/move_group/monitored_planning_scene
/move_group/motion_plan_request
/move_group/plan_execution/parameter_descriptions
/move_group/plan_execution/parameter_updates
/move_group/planning_pipelines/ompl/ompl/parameter_descriptions
/move_group/planning_pipelines/ompl/ompl/parameter_updates
/move_group/planning_scene_monitor/parameter_descriptions
/move_group/planning_scene_monitor/parameter_updates
/move_group/result
/move_group/sense_for_plan/parameter_descriptions
/move_group/sense_for_plan/parameter_updates
/move_group/status
/move_group/trajectory_execution/parameter_descriptions
/move_group/trajectory_execution/parameter_updates
/odom
/pickup/cancel
/pickup/feedback
/pickup/goal
/pickup/result
/pickup/status
/place/cancel
/place/feedback
/place/goal
/place/result
/place/status
/planning_scene
/planning_scene_world
/recognized_object_array
/rosout
/rosout_agg
/rviz_3b17871017fd_16156_6387028559664899485/motionplanning_planning_scene_monitor/parameter_descriptions
/rviz_3b17871017fd_16156_6387028559664899485/motionplanning_planning_scene_monitor/parameter_updates
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full
/scan
/sequence_move_group/cancel
/sequence_move_group/feedback
/sequence_move_group/goal
/sequence_move_group/result
/sequence_move_group/status
/slam_gmapping/entropy
/tf
/tf_static
/trajectory_execution_event

```

in particular, here are the informations about the most significant topics for the system (I didn't configure RViz yet)

```
root@3b17871017fd:~# rostopic info /map
Type: nav_msgs/OccupancyGrid

Publishers: 
 * /slam_gmapping (http://3b17871017fd:41233/)

Subscribers: 
 * /move_base (http://3b17871017fd:39225/)


root@3b17871017fd:~# rostopic info /map
Type: nav_msgs/OccupancyGrid

Publishers: 
 * /slam_gmapping (http://3b17871017fd:41233/)

Subscribers: 
 * /move_base (http://3b17871017fd:39225/)


root@3b17871017fd:~# rostopic info /move_base/global_costmap/costmap
Type: nav_msgs/OccupancyGrid

Publishers: 
 * /move_base (http://3b17871017fd:39225/)

Subscribers: None


root@3b17871017fd:~# rostopic info /move_base/local_costmap/costmap
Type: nav_msgs/OccupancyGrid

Publishers: 
 * /move_base (http://3b17871017fd:39225/)

Subscribers: None


root@3b17871017fd:~# rostopic info /scan 
Type: sensor_msgs/LaserScan

Publishers: 
 * /gazebo (http://3b17871017fd:33139/)

Subscribers: 
 * /rviz_3b17871017fd_16156_6387028559664899485 (http://3b17871017fd:38527/)
 * /slam_gmapping (http://3b17871017fd:41233/)
 * /move_base (http://3b17871017fd:39225/)

```

### RViz configuration

Here's my RViz configuration for the project, starting from the automatically generated file `robocluedo_robot/launch/moveit.rviz`:

- add (by topic) *laser scan* ACTIVATED
- add (by display type) *RobotModel* UNCHECK 
	just to be sure that the model has been correctly loaded
- add (by topic) *map* ACTIVATED
- add (by topic) *global_costmap/map* ACTIVATED
- add (by topic) *loca_costmap/map* ACTIVATED
- add (by display type) *PlanningScene* ACTIVATED
- add (by topic) *move_base/current_goal/pose* ACTIVATED
- add (by topic) *move_base/trajectory_planner_ros/local_plan/path* ACTIVATED
- add (by topic) *move_base/trajectory_planner_ros/global_plan/path* ACTIVATED
- add (tools bar, new tool) *rviz/setGoal*
- add (by display type) Axes

If you just want to deal with the navstack, there's a working rviz file `robocluedo_robot/sim_nav_stack.rviz`. 

## ISSUE - disk requirement over 1Gb

sometimes, one launch file could issue this "red" warning:

```
WARNING: disk usage in log directory [/root/.ros/log] is over 1GB.
It's recommended that you use the 'rosclean' command.
```

this means that the system is occupying a lot of memory in logging. In my case,

```
# rosclean check
1.3G ROS node logs

# rosclean purge
Purging ROS node logs.
PLEASE BE CAREFUL TO VERIFY THE COMMAND BELOW!
Okay to perform:

rm -rf /root/.ros/log
(y/n)?
y

# rosclean check
	#	...empty...
```

the third command doesn't return. 

### rosclean -- help

```
# rosclean --help
usage: rosclean [-h] {check,purge} ...

positional arguments:
  {check,purge}
    check        Check usage of log files
    purge        Remove log files

optional arguments:
  -h, --help     show this help message and exit
```

## Set a world

first of all, a small update of the file *gazebo.launch* is required. For your convenience, here's the complete file: just select all, and replace with this. 

```xml
<?xml version="1.0"?>
<launch>
  <arg name="world_file_path" />
  <arg name="paused" default="false"/>
  <arg name="gazebo_gui" default="true"/>
  <arg name="initial_joint_positions" doc="Initial joint configuration of the robot"
       default=" -J arm_joint_01 0 -J arm_joint_02 0 -J arm_joint_03 0 -J arm_joint_04 0"/>

  <!-- startup simulated world -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" default="$(find robocluedo_robot)/worlds/$(arg world_file_path)"/>
    <arg name="paused" value="true"/>
    <arg name="gui" value="$(arg gazebo_gui)"/>
  </include>

  <!-- send robot urdf to param server -->
  <param name="robot_description" textfile="$(find robocluedo_movement_controller)/urdf/robocluedo.urdf" />

  <!-- unpause only after loading robot model -->
  <arg name="unpause" value="$(eval '' if arg('paused') else '-unpause')" />
  <!-- push robot_description to factory and spawn robot in gazebo at the origin, change x,y,z arguments to spawn in a different position -->
  <arg name="world_pose" value="-x 0 -y 0 -z 0" />
  <node name="spawn_gazebo_model" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -model robot $(arg unpause) $(arg world_pose) $(arg initial_joint_positions)"
    respawn="false" output="screen" />

  <!-- Load joint controller parameters for Gazebo -->
  <rosparam file="$(find robocluedo_robot)/config/gazebo_controllers.yaml" />
  <!-- Spawn Gazebo ROS controllers -->
  <node name="gazebo_controller_spawner" pkg="controller_manager" type="spawner" respawn="false" output="screen" args="joint_state_controller" />
  <!-- Load ROS controllers -->
  <include file="$(dirname)/ros_controllers.launch"/>

</launch>
```

I just added a parameter `world_file_path` containing the path of the `.world` file to run. Remember to specify this parameter from command line in this way:

```bash
roslaunch robocluedo_robot gazebo.launch world_file_path:=square_room.world

```

**this arg is mandatory**. The package contains two files:

> `square_room.world` : a simple 5mx5m room

the robot will be spawned exactly in the middle of the square. 

```bash
roslaunch robocluedo_robot gazebo.launch world_file_path:=square_room.world

```

> `house` : a small maze, or a very raw representation of a house

the robot will be spawned exactly in the middle of the ambient. 

```bash
roslaunch robocluedo_robot gazebo.launch world_file_path:=house.world

```

**note that** the system won't raise a error if the name of the world file is wrong! the robot is spawned in a empty world with no error notifications or warnings. 

### RViz config file selection

another update, for choosing the configuration file to use with RViz. As before, copy and replace! file `robocluedo_robot/launch/demo.launch`. 

```xml
<launch>

  <!-- RViz config file -->
  <arg name="rviz_config_file" />
  
  <!-- specify the planning pipeline -->
  <arg name="pipeline" default="ompl" />

  <!-- By default, we do not start a database (it can be large) -->
  <arg name="db" default="false" />
  <!-- Allow user to specify database location -->
  <arg name="db_path" default="$(find robocluedo_robot)/default_warehouse_mongo_db" />

  <!-- By default, we are not in debug mode -->
  <arg name="debug" default="false" />

  <!-- By default, we will load or override the robot_description -->
  <arg name="load_robot_description" default="true"/>

  <!-- Choose controller manager: fake, simple, or ros_control -->
  <arg name="moveit_controller_manager" default="fake" />
  <!-- Set execution mode for fake execution controllers -->
  <arg name="fake_execution_type" default="interpolate" />

  <!-- By default, hide joint_state_publisher's GUI in 'fake' controller_manager mode -->
  <arg name="use_gui" default="false" />
  <arg name="use_rviz" default="true" />

  <!-- If needed, broadcast static tf for robot root -->



  <group if="$(eval arg('moveit_controller_manager') == 'fake')">
    <!-- We do not have a real robot connected, so publish fake joint states via a joint_state_publisher
         MoveIt's fake controller's joint states are considered via the 'source_list' parameter -->
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" unless="$(arg use_gui)">
      <rosparam param="source_list">[move_group/fake_controller_joint_states]</rosparam>
    </node>
    <!-- If desired, a GUI version is available allowing to move the simulated robot around manually
         This corresponds to moving around the real robot without the use of MoveIt. -->
    <node name="joint_state_publisher" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" if="$(arg use_gui)">
      <rosparam param="source_list">[move_group/fake_controller_joint_states]</rosparam>
    </node>

    <!-- Given the published joint states, publish tf for the robot links -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" />
  </group>

  <!-- Run the main MoveIt executable without trajectory execution (we do not have controllers configured by default) -->
  <include file="$(dirname)/move_group.launch">
    <arg name="allow_trajectory_execution" value="true"/>
    <arg name="moveit_controller_manager" value="$(arg moveit_controller_manager)" />
    <arg name="fake_execution_type" value="$(arg fake_execution_type)"/>
    <arg name="info" value="true"/>
    <arg name="debug" value="$(arg debug)"/>
    <arg name="pipeline" value="$(arg pipeline)"/>
    <arg name="load_robot_description" value="$(arg load_robot_description)"/>
  </include>

  <!-- Run Rviz and load the default config to see the state of the move_group node -->
  <include file="$(dirname)/moveit_rviz.launch" if="$(arg use_rviz)">
    <arg name="rviz_config" value="$(find robocluedo_robot)/config/rviz/$(arg rviz_config_file)"/>
    <arg name="debug" value="$(arg debug)"/>
  </include>

  <!-- If database loading was enabled, start mongodb as well -->
  <include file="$(dirname)/default_warehouse_db.launch" if="$(arg db)">
    <arg name="moveit_warehouse_database_path" value="$(arg db_path)"/>
  </include>

</launch>
```

The new folder for the configuration files is `robocluedo_robot/config/rviz/` which contains two configuration files:

- `moveit.launch.rviz` : the file generated by moveit setup assistant, plus some other features

```bash
roslaunch robocluedo_robot demo.launch rviz_config_file:=moveit.rviz

```

- `sim_nav_stack.rviz` : it allows to interact via RViz with the navigation stack

```bash
roslaunch robocluedo_robot demo.launch rviz_config_file:=sim_nav_stack.rviz

```

## about tuning

now, the system must be tuned with respect to the problem here, otherwise the robot will perform very poorly, and seldom it will remain stuck in a position. In my case, the robot, once an objective is provided, *keeps going forward and backward* remaining approximately at the same point. Another signal is that the robot seldom uses a command which is opposite to the one sent by the local planner: the planner says to turn right for instance, and the robot turns left. 

- [a good guide](https://wiki.ros.org/navigation/Tutorials/Navigation%20Tuning%20Guide) about the tuning
- see also [here](https://wiki.ros.org/navigation/Tutorials/RobotSetup)
- see [this post](https://answers.ros.org/question/192109/costmap_2d-observation_sources-expected_update_rate/) about the source parameter
- very useful is [teleop_twist_key](http://wiki.ros.org/teleop_twist_joy), and here the [gitHub page](https://github.com/ros-teleop/teleop_twist_joy)

### run teleop_twist_key

**install**:

```bash
sudo apt-get install ros-noetic-teleop-twist-keyboard
```

**run the node**:

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py

```

here's the help of this utility:

```
# rosrun teleop_twist_keyboard teleop_twist_keyboard.py

Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit

currently:	speed 0.5	turn 1.0 
```

## A VERY UNFORGETTABLE ISSUE with differential_drive_plugin

my problem was this:

- the local planner says to turn in one direction ...
- ...but the robot *turns in another direction*
- after some tryings, the robot start moving randomly, and in particular going forward and backward in the same position

maybe the problem was in the differential drive ... I checked, but there's no bug. here is the code:

```xml
    <gazebo>
        <plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive_controller">
            <legacyMode>true</legacyMode>
            <alwaysOn>true</alwaysOn>
            <updateRate>20</updateRate>
            <leftJoint>joint_left_wheel</leftJoint>
            <rightJoint>joint_right_wheel</rightJoint>
            <wheelSeparation>0.3</wheelSeparation>
            <wheelDiameter>0.2</wheelDiameter>
            <torque>0.1</torque>
            <commandTopic>cmd_vel</commandTopic>
            <odometryTopic>odom</odometryTopic>
            <odometryFrame>odom</odometryFrame>
            <robotBaseFrame>base_link</robotBaseFrame>
            <rosDebugLevel>na</rosDebugLevel>
            <publishWheelTF>true</publishWheelTF>
            <publishWheelJointState>true</publishWheelJointState>
            <wheelAcceleration>0</wheelAcceleration>
            <wheelTorque>5</wheelTorque>
            <odometrySource>1</odometrySource>
            <publishTf>1</publishTf>
        </plugin>
    </gazebo>
```

... idea: what about to swap left and right?

```xml
    <gazebo>
        <plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive_controller">
            <legacyMode>true</legacyMode>
            <alwaysOn>true</alwaysOn>
            <updateRate>20</updateRate>
            
            <leftJoint>joint_right_wheel</leftJoint>
            <rightJoint>joint_left_wheel</rightJoint>
            
            <wheelSeparation>0.3</wheelSeparation>
            <wheelDiameter>0.2</wheelDiameter>
            <torque>0.1</torque>
            <commandTopic>cmd_vel</commandTopic>
            <odometryTopic>odom</odometryTopic>
            <odometryFrame>odom</odometryFrame>
            <robotBaseFrame>base_link</robotBaseFrame>
            <rosDebugLevel>na</rosDebugLevel>
            <publishWheelTF>true</publishWheelTF>
            <publishWheelJointState>true</publishWheelJointState>
            <wheelAcceleration>0</wheelAcceleration>
            <wheelTorque>5</wheelTorque>
            <odometrySource>1</odometrySource>
            <publishTf>1</publishTf>
        </plugin>
    </gazebo>
```

and ... TADA! it works!

![**MAGIC.**](./magic.gif)

![**MIND BLOWING**](./mindblowing.gif)

## ISSUE MoveIt -- the arm doesn't move

with this configuration ... the navigation stack moves beautifully in the space ... but the arm does not! Both with gazebo+RViz (a lot of disturbances here) and with RViz only (the planner succeed but not the execution phase, in which the arm doesn't move at all). 

unfortunately, finding satisfying answers to this issue in the Internet is almost impossible. 

### trying -- the robot moves too slow

- [here](https://answers.ros.org/question/346203/why-is-my-robot-moving-so-slow-with-moveit/) is a post about it

the new config file:

```yaml
# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed

# For beginners, we downscale velocity and acceleration limits.
# You can always specify higher scaling factors (<= 1.0) in your motion requests.  # Increase the values below to 1.0 to always move at maximum speed.
default_velocity_scaling_factor: 1
default_acceleration_scaling_factor: 0.1

# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]
# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]
joint_limits:
  arm_joint_01:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: false
    max_acceleration: 0
  arm_joint_02:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: false
    max_acceleration: 0
  arm_joint_03:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: false
    max_acceleration: 0
  arm_joint_04:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: false
    max_acceleration: 0

```

and here is the new file 'ros_controllers.yaml':

```yaml
arm_group_controller:
  type: effort_controllers/JointTrajectoryController
  joints:
    - arm_joint_01
    - arm_joint_02
    - arm_joint_03
    - arm_joint_04
  gains:
    arm_joint_01:
      p: 10
      d: 0
      i: 0
      i_clamp: 0
    arm_joint_02:
      p: 10
      d: 0
      i: 0
      i_clamp: 0
    arm_joint_03:
      p: 10
      d: 0
      i: 0
      i_clamp: 0
    arm_joint_04:
      p: 10
      d: 0
      i: 0
      i_clamp: 0

```

now the robot moves in RViz only, but not in Gazebo+RViz...

### trying again -- gazebo is not listening

- as suggested [here](https://answers.ros.org/question/310234/moveit-trajectory-not-received-by-gazebo-joint-controllers/)

... but this is not my problem: using `rqt_graph`, the graph is correctly made. Gazebo is communicating correctlywith moveit.

### trying again -- arm is unstable

- maybe is it a matter of physics? see [this post](https://answers.ros.org/question/43327/controller-makes-robot-unstable-in-gazebo/) ... but useless
- another possible way to solve the issue [here](https://answers.ros.org/question/374639/moveit_servo-wont-move-arm-in-gazebo-only-oscillates/) ... but neither this works
- [here](https://answers.ros.org/question/157261/manipulator-shakeswobbles/?answer=157976?answer=157976#post-id-157976) is a similar issue

### the last error (right now)

```
[ERROR] [1658042520.742037100, 189.816000000]: 
Invalid Trajectory: start point deviates from current robot state more than 0.01
joint 'arm_joint_02': expected: 0.0323387, current: 0
```

## A copy of my URDF file

automatically generated by the xacro utility

```xml
<?xml version="1.0" ?>
<!-- =================================================================================== -->
<!-- |    This document was autogenerated by xacro from /root/ros_ws/src/robot_urdf/urdf/m2wr_arm2.xacro | -->
<!-- |    EDITING THIS FILE BY HAND IS NOT RECOMMENDED                                 | -->
<!-- =================================================================================== -->
<robot name="m2wr">
    <material name="black">
        <color rgba="0.0 0.0 0.0 1.0" />
    </material>
    <material name="blue">
        <color rgba="0.203125 0.23828125 0.28515625 1.0" />
    </material>
    <material name="green">
        <color rgba="0.0 0.8 0.0 1.0" />
    </material>
    <material name="grey">
        <color rgba="0.2 0.2 0.2 1.0" />
    </material>
    <material name="orange">
        <color rgba="1.0 0.423529411765 0.0392156862745 1.0" />
    </material>
    <material name="brown">
        <color rgba="0.870588235294 0.811764705882 0.764705882353 1.0" />
    </material>
    <material name="red">
        <color rgba="0.80078125 0.12890625 0.1328125 1.0" />
    </material>
    <material name="white">
        <color rgba="1.0 1.0 1.0 1.0" />
    </material>
    <gazebo reference="base_link">
        <material>Gazebo/Orange</material>
    </gazebo>
    <gazebo reference="link_left_wheel">
        <material>Gazebo/Blue</material>
    </gazebo>
    <gazebo reference="link_right_wheel">
        <material>Gazebo/Blue</material>
    </gazebo>
    <gazebo reference="arm_base_link">
        <material>Gazebo/Red</material>
    </gazebo>
    <gazebo reference="arm_link_01">
        <material>Gazebo/Red</material>
    </gazebo>
    <gazebo reference="arm_link_02">
        <material>Gazebo/Red</material>
    </gazebo>
    <gazebo reference="arm_link_03">
        <material>Gazebo/Red</material>
    </gazebo>
    <gazebo reference="cluedo_link">
        <material>Gazebo/Red</material>
    </gazebo>
    <gazebo reference="right_grip_link">
        <material>Gazebo/White</material>
    </gazebo>
    <gazebo reference="left_grip_link">
        <material>Gazebo/White</material>
    </gazebo>
    <gazebo reference="arm_joint_02">
        <provideFeedback>true</provideFeedback>
    </gazebo>
    <gazebo>
        <plugin filename="libgazebo_ros_ft_sensor.so" name="ft_sensor">
            <updateRate>100.0</updateRate>
            <topicName>ft_sensor_topic</topicName>
            <jointName>arm_joint_02</jointName>
        </plugin>
    </gazebo>
    <gazebo>
        <plugin filename="libgazebo_ros_diff_drive.so" name="differential_drive_controller">
            <legacyMode>true</legacyMode>
            <alwaysOn>true</alwaysOn>
            <updateRate>20</updateRate>
            
            <!-- MAGIC. -->
            <leftJoint>joint_right_wheel</leftJoint>
            <rightJoint>joint_left_wheel</rightJoint>
            <!-- MAGIC. -->
            
            <wheelSeparation>0.3</wheelSeparation>
            <wheelDiameter>0.2</wheelDiameter>
            <torque>0.1</torque>
            <commandTopic>cmd_vel</commandTopic>
            <odometryTopic>odom</odometryTopic>
            <odometryFrame>odom</odometryFrame>
            <robotBaseFrame>base_link</robotBaseFrame>
            <rosDebugLevel>na</rosDebugLevel>
            <publishWheelTF>true</publishWheelTF>
            <publishWheelJointState>true</publishWheelJointState>
            <wheelAcceleration>0</wheelAcceleration>
            <wheelTorque>5</wheelTorque>
            <odometrySource>1</odometrySource>
            <publishTf>1</publishTf>
        </plugin>
    </gazebo>
    <gazebo reference="laser">
        <sensor name="head_hokuyo_sensor" type="ray">
            <pose>0 0 0 0 0 0</pose>
            <visualize>false</visualize>
            <update_rate>20</update_rate>
            <ray>
                <scan>
                    <horizontal>
                        <samples>720</samples>
                        <resolution>1</resolution>
                        <min_angle>-1.570796</min_angle>
                        <max_angle>1.570796</max_angle>
                    </horizontal>
                </scan>
                <range>
                    <min>0.10</min>
                    <max>10.0</max>
                    <resolution>0.01</resolution>
                </range>
                <noise>
                    <type>gaussian</type>
                    <mean>0.0</mean>
                    <stddev>0.01</stddev>
                </noise>
            </ray>
            <plugin filename="libgazebo_ros_laser.so" name="gazebo_ros_head_hokuyo_controller">
                <topicName>/scan</topicName>
                <frameName>laser</frameName>
            </plugin>
        </sensor>
    </gazebo>
    <gazebo>
        <plugin filename="libgazebo_ros_control.so" name="gazebo_ros_control">
            <robotNamespace>m2wr</robotNamespace>
            <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
            <legacyModeNS>true</legacyModeNS>
        </plugin>
    </gazebo>
    <link name="base_link">
        <!-- pose and inertial -->
        <pose>0 0 0.1 0 0 0</pose>
        
<inertial>
            <mass value="5" />
            <origin rpy="0 0 0" xyz="0 0 0.1" />
            <inertia ixx="0.0395416666667" ixy="0" ixz="0" iyy="0.106208333333" iyz="0" izz="0.106208333333" />
        </inertial>

        <collision name="collision_chassis">
            <geometry>
                <box size="0.5 0.3 0.07" />
            </geometry>
        </collision>
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0" />
            <geometry>
                <box size="0.5 0.3 0.07" />
            </geometry>
            <material name="blue" />
        </visual>
        <!-- caster front -->
        <collision name="caster_front_collision">
            <origin rpy=" 0 0 0" xyz="0.35 0 -0.05" />
            <geometry>
                <sphere radius="0.05" />
            </geometry>
            <surface>
                <friction>
                    <ode>
                        <mu>0</mu>
                        <mu2>0</mu2>
                        <slip1>1.0</slip1>
                        <slip2>1.0</slip2>
                    </ode>
                </friction>
            </surface>
        </collision>
        <visual name="caster_front_visual">
            <origin rpy=" 0 0 0" xyz="0.2 0 -0.05" />
            <geometry>
                <sphere radius="0.05" />
            </geometry>
        </visual>
    </link>
    <!-- Create wheels -->
    <link name="link_right_wheel">
        
<inertial>
            <mass value="0.2" />
            <origin rpy="0 1.5707 1.5707" xyz="0 0 0" />
            <inertia ixx="0.000526666666667" ixy="0" ixz="0" iyy="0.000526666666667" iyz="0" izz="0.001" />
        </inertial>

        <collision name="link_right_wheel_collision">
            <origin rpy="0 1.5707 1.5707" xyz="0 0 0" />
            <geometry>
                <cylinder length="0.04" radius="0.1" />
            </geometry>
        </collision>
        <visual name="link_right_wheel_visual">
            <origin rpy="0 1.5707 1.5707" xyz="0 0 0" />
            <geometry>
                <cylinder length="0.04" radius="0.1" />
            </geometry>
        </visual>
    </link>
    <link name="link_left_wheel">
        
<inertial>
            <mass value="0.2" />
            <origin rpy="0 1.5707 1.5707" xyz="0 0 0" />
            <inertia ixx="0.000526666666667" ixy="0" ixz="0" iyy="0.000526666666667" iyz="0" izz="0.001" />
        </inertial>

        <collision name="link_right_wheel_collision">
            <origin rpy="0 1.5707 1.5707" xyz="0 0 0" />
            <geometry>
                <cylinder length="0.04" radius="0.1" />
            </geometry>
        </collision>
        <visual name="link_left_wheel_visual">
            <origin rpy="0 1.5707 1.5707" xyz="0 0 0" />
            <geometry>
                <cylinder length="0.04" radius="0.1" />
            </geometry>
        </visual>
    </link>
    <joint name="joint_right_wheel" type="continuous">
        <origin rpy="0 0 0" xyz="-0.05 0.15 0" />
        <child link="link_right_wheel" />
        <parent link="base_link" />
        <axis rpy="0 0 0" xyz="0 1 0" />
        <limit effort="10000" velocity="1000" />
        <joint_properties damping="1.0" friction="1.0" />
    </joint>
    <joint name="joint_left_wheel" type="continuous">
        <origin rpy="0 0 0" xyz="-0.05 -0.15 0" />
        <child link="link_left_wheel" />
        <parent link="base_link" />
        <axis rpy="0 0 0" xyz="0 1 0" />
        <limit effort="10000" velocity="1000" />
        <joint_properties damping="1.0" friction="1.0" />
    </joint>
    <link name="laser">
        
<inertial>
            <origin rpy="0 0 0" xyz="0 0 0" />
            <mass value="1" />
            <inertia ixx="0.0014583333333333334" ixy="0" ixz="0" iyy="0.0014583333333333334" iyz="0" izz="0.0012500000000000002" />
        </inertial>

        <visual>
            <origin rpy="0 0 0" xyz="0 0 0" />
            <geometry>
                <cylinder length="0.1" radius="0.05" />
            </geometry>
            <material name="white" />
        </visual>
        <collision>
            <origin rpy="0 0 0" xyz="0 0 0" />
            <geometry>
                <cylinder length="0.1" radius="0.05" />
            </geometry>
        </collision>
    </link>
    <joint name="joint_laser" type="fixed">
        <origin rpy="0 0 0" xyz="0.15 0 0.05" />
        <parent link="base_link" />
        <child link="laser" />
    </joint>
    <!--arm definition -->
    <link name="arm_base_link">
        
<inertial>
            <mass value="0.1" />
            <origin rpy="0 0 0" xyz="0 0 0" />
            <inertia ixx="0.0000416666667" ixy="0" ixz="0" iyy="0.0000416666667" iyz="0" izz="0.0000416666667" />
        </inertial>

        <collision>
            <origin rpy="0 0 0" xyz="0 0 0" />
            <geometry>
                <box size="0.1 0.1 0.1" />
            </geometry>
        </collision>
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0" />
            <geometry>
                <box size="0.1 0.1 0.1" />
            </geometry>
            <material name="red" />
        </visual>
    </link>
    <joint name="arm_base_to_base" type="fixed">
        <origin rpy="0 0 0" xyz="0 0 0.085" />
        <parent link="base_link" />
        <child link="arm_base_link" />
    </joint>
    <link name="arm_link_01">
        
<inertial>
            <mass value="0.025" />
            <origin rpy="0 0 0" xyz="0 0 0" />
            <inertia ixx="0.000010279" ixy="0" ixz="0" iyy="0.000010279" iyz="0" izz="0.000007225" />
        </inertial>

        <collision>
            <origin rpy="0 0 0" xyz="0 0 0" />
            <geometry>
                <cylinder length="0.4" radius="0.04" />
            </geometry>
        </collision>
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0" />
            <geometry>
                <cylinder length="0.4" radius="0.04" />
            </geometry>
            <material name="red" />
        </visual>
    </link>
    <joint name="arm_joint_01" type="revolute">
        <axis xyz="0 0 1" />
        <limit effort="1000.0" lower="-3.14" upper="3.14" velocity="0.2" />
        <origin rpy="0 0 0" xyz="0 0 0.15" />
        <parent link="arm_base_link" />
        <child link="arm_link_01" />
    </joint>
    <link name="arm_link_02">
        
<inertial>
            <mass value="0.015" />
            <origin rpy="0 0 0" xyz="0 0 0" />
            <inertia ixx="0.000010279" ixy="0" ixz="0" iyy="0.000010279" iyz="0" izz="0.000007225" />
        </inertial>

        <collision>
            <origin rpy="0 0 0" xyz="0 0 0.2" />
            <geometry>
                <cylinder length="0.4" radius="0.04" />
            </geometry>
        </collision>
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0.2" />
            <geometry>
                <cylinder length="0.4" radius="0.04" />
            </geometry>
            <material name="red" />
        </visual>
    </link>
    <joint name="arm_joint_02" type="revolute">
        <axis xyz="1 0 0" />
        <limit effort="1000.0" lower="-1.8" upper="1.8" velocity="0.2" />
        <origin rpy="0 0 0" xyz="0 0 0.2" />
        <parent link="arm_link_01" />
        <child link="arm_link_02" />
    </joint>
    <link name="arm_link_03">
        
<inertial>
            <mass value="0.015" />
            <origin rpy="0 0 0" xyz="0 0 0" />
            <inertia ixx="0.000010279" ixy="0" ixz="0" iyy="0.000010279" iyz="0" izz="0.000007225" />
        </inertial>

        <collision>
            <origin rpy="0 0 0" xyz="0 0 0.2" />
            <geometry>
                <cylinder length="0.4" radius="0.04" />
            </geometry>
        </collision>
        <visual>
            <origin rpy="0 0 0" xyz="0 0 0.2" />
            <geometry>
                <cylinder length="0.4" radius="0.04" />
            </geometry>
            <material name="red" />
        </visual>
    </link>
    <joint name="arm_joint_03" type="revolute">
        <axis xyz="1 0 0" />
        <limit effort="1000.0" lower="-1.8" upper="1.8" velocity="0.2" />
        <origin rpy="0 0 0" xyz="0 0 0.4" />
        <parent link="arm_link_02" />
        <child link="arm_link_03" />
    </joint>
    <link name="cluedo_link">
        
<inertial>
            <mass value="0.015" />
            <origin rpy="0 0 0" xyz="0 0 0" />
            <inertia ixx="0.000010279" ixy="0" ixz="0" iyy="0.000010279" iyz="0" izz="0.000007225" />
        </inertial>

        <collision>
            <origin rpy="0 0 1.57" xyz="0 0 0.04" />
            <geometry>
                <box size="0.034 0.2 0.08" />
            </geometry>
        </collision>
        <visual>
            <origin rpy="0 0 1.57" xyz="0 0 0.04" />
            <geometry>
                <box size="0.034 0.2 0.08" />
            </geometry>
            <material name="red" />
        </visual>
    </link>
    <joint name="arm_joint_04" type="revolute">
        <axis xyz="1 0 0" />
        <limit effort="1000.0" lower="-1.8" upper="1.8" velocity="0.5" />
        <origin rpy="0 0 0" xyz="0 0 0.4" />
        <parent link="arm_link_03" />
        <child link="cluedo_link" />
    </joint>

    <link name="right_grip_link">
        
<inertial>
            <mass value="0.01" />
            <origin rpy="0 0 0" xyz="0 0 0" />
            <inertia ixx="0.000010279" ixy="0" ixz="0" iyy="0.000010279" iyz="0" izz="0.000007225" />
        </inertial>

        <collision>
            <origin rpy="0 3.1415 -1.57" xyz="-0.032 0.048 0.04" />
            <geometry>
                <box size="0.06 0.02 0.08" />
            </geometry>
        </collision>
        <visual>
            <origin rpy="0 3.1415 -1.57" xyz="-0.032 0.048 0.04" />
            <geometry>
                <box size="0.06 0.02 0.08" />
            </geometry>
            <material name="white" />
        </visual>
    </link>
    <joint name="right_grip_joint" type="prismatic">
        <axis xyz="1 0 0" />
        <limit effort="1000.0" lower="-0.04" upper="0.035" velocity="6.5" />
        <origin rpy="0 0 0" xyz="-0.015 0 0.0" />
        <parent link="cluedo_link" />
        <child link="right_grip_link" />
    </joint>
    <link name="left_grip_link">
        
<inertial>
            <mass value="0.01" />
            <origin rpy="0 0 0" xyz="0 0 0" />
            <inertia ixx="0.000010279" ixy="0" ixz="0" iyy="0.000010279" iyz="0" izz="0.000007225" />
        </inertial>

        <collision>
            <origin rpy="0 3.1415 -1.57" xyz="0.06 0.048 0.04" />
            <geometry>
                <box size="0.06 0.02 0.08" />
            </geometry>
        </collision>
        <visual>
            <origin rpy="0 3.1415 -1.57" xyz="0.06 0.048 0.04" />
            <geometry>
                <box size="0.06 0.02 0.08" />
            </geometry>
            <material name="white" />
        </visual>
    </link>
    <joint name="left_grip_joint" type="prismatic">
        <axis xyz="1 0 0" />
        <limit effort="1000.0" lower="-0.035" upper="0.04" velocity="6.5" />
        <origin rpy="0 0 0" xyz="-0.015 0 0.0" />
        <parent link="cluedo_link" />
        <child link="left_grip_link" />
    </joint>
    
<transmission name="arm_joint_01_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="arm_joint_01">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor1">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalInterface>1</mechanicalInterface>
        </actuator>
    </transmission>
    <transmission name="arm_joint_02_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="arm_joint_02">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor2">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalInterface>1</mechanicalInterface>
        </actuator>
    </transmission>
    <transmission name="arm_joint_03_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="arm_joint_03">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor3">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalInterface>1</mechanicalInterface>
        </actuator>
    </transmission>
    <transmission name="arm_joint_04_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="arm_joint_04">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor4">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalInterface>1</mechanicalInterface>
        </actuator>
    </transmission>
    
    <transmission name="left_grip_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="left_grip_joint">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor5">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalInterface>1</mechanicalInterface>
        </actuator>
    </transmission>
    <transmission name="right_grip_trans">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="right_grip_joint">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="motor6">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalInterface>1</mechanicalInterface>
        </actuator>
    </transmission>
    <transmission name="trans_joint_right_wheel">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint_right_wheel">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="joint_right_wheel_motor">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
    <transmission name="trans_joint_left_wheel">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="joint_left_wheel">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="joint_left_wheel_motor">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
    <transmission name="trans_arm_joint_01">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="arm_joint_01">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="arm_joint_01_motor">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
    <transmission name="trans_arm_joint_02">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="arm_joint_02">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="arm_joint_02_motor">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
    <transmission name="trans_arm_joint_03">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="arm_joint_03">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="arm_joint_03_motor">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
    <transmission name="trans_arm_joint_04">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="arm_joint_04">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="arm_joint_04_motor">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
    <transmission name="trans_right_grip_joint">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="right_grip_joint">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="right_grip_joint_motor">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
    <transmission name="trans_left_grip_joint">
        <type>transmission_interface/SimpleTransmission</type>
        <joint name="left_grip_joint">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
        </joint>
        <actuator name="left_grip_joint_motor">
            <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
            <mechanicalReduction>1</mechanicalReduction>
        </actuator>
    </transmission>
    <gazebo>
        <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
            <robotNamespace>/</robotNamespace>
        </plugin>
    </gazebo>
</robot>

```

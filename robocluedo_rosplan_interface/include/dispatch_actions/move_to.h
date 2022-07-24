
/********************************************//**
*  
* @file move_to.h
* 
* @brief implementation of the action (move-to ?from ?to )
* 
* @authors Francesco Ganci
* @version v1.0
* 
* ... more details
* 
***********************************************/

#ifndef __H_KCL_MOVE_TO__
#define __H_KCL_MOVE_TO__ "__H_MOVE_TO__"

#include "ros/ros.h"
#include "knowledge_base_tools/robocluedo_kb_tools.h"
#include "robocluedo_rosplan_action_interface/RPActionInterface.h"
#include <tf/tf.h>

#include "robocluedo_rosplan_interface_msgs/ActionFeedback.h"
#include "dispatch_actions/feedback_manager.h"

#define NODE_NAME "move_to"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#include "diagnostic_msgs/KeyValue.h"
/*
string key
string value 
*/

#include "rosplan_dispatch_msgs/ActionDispatch.h"
/*
#actionDispatch message
int32 action_id
int32 plan_id
string name
diagnostic_msgs/KeyValue[] parameters
float32 duration
float32 dispatch_time
*/

#include "visualization_msgs/MarkerArray.h"
// http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html
/*
Marker[] markers
*/

#include "visualization_msgs/Marker.h"
/*
uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11
uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

std_msgs/Header header
string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
geometry_msgs/Vector3 scale
std_msgs/ColorRGBA color
duration lifetime
bool frame_locked
geometry_msgs/Point[] points
std_msgs/ColorRGBA[] colors
string text
string mesh_resource
bool mesh_use_embedded_materials
*/

#include "geometry_msgs/Pose.h"
/*
Point position {x, y, z}
Quaternion orientation {x, y, z, w}
*/

#include "geometry_msgs/Vector3.h"
// x, y, z

#include "robocluedo_rosplan_interface_msgs/NavigationCommand.h"
/*
## file 'NavigationCommand.srv'
# the ROSPlan expects that a external node implements this service

## REQUEST

# only x and y, planar navigation
geometry_msgs/Point target_2d

# compute or not the orientation? 
#    (compute the orientation with the marker if true)
bool look_to_marker

# the marker to reach (if look_to_marker)
geometry_msgs/Point marker

---

## RESPONSE

# success 
bool success
*/

#include <vector>
#include <unistd.h>
#include <cmath>

#define Q_SZ 1

// markers update
#define TOPIC_MARKER "/visualization_marker"

// navigation command service
#define SERVICE_NAV "/robocluedo/navigation_command"
#define TIMEOUT_NAV 30


namespace KCL_rosplan
{

/********************************************//**
 * 
 * \class RP_move_to
 * 
 * \brief implementation of the action move_to
 * 
 * ... more details
 * 
 ***********************************************/
class RP_move_to : public RPActionInterface, public robocluedo_kb_tools
{
public:
	
	/// empty constructor (don't use it!)
	RP_move_to( );
	
	/********************************************//**
	 *  
	 * \brief class constructor
	 * 
	 * ... more details
	 * 
	 * @param nh (ros::NodeHandle) the handle of the node
	 * @param debug_mode verbose print or not?
	 * 
	 * @todo open the publisher to the mission manager?
	 * 
	 * @todo read the waypoints and their poses directly from the markers 
	 * topic. 
	 * 
	 ***********************************************/
	RP_move_to( ros::NodeHandle &nh, bool debug_mode = KB_TOOLS_DEFAULT_DEBUG_MODE );
	
	/********************************************//**
	 *  
	 * \brief class destructor
	 * 
	 ***********************************************/
	~RP_move_to( );
	
	/********************************************//**
	 *  
	 * \brief listen to and process action_dispatch topic
	 * 
	 * here's the procedure: 
	 * <ol>
	 * <li></li>
	 * <li></li>
	 * <li></li>
	 * </ol>
	 * 
	 * @param msg the action to execute
	 * 
	 * @returns true if the action succeeded, false in case of problems
	 * 
	 * @note the node can communicate with a node in charge to orchestrate
	 * the mission and, in particular, to take a decision when a replan action
	 * becomes necessary. 
	 * 
	 ***********************************************/
	bool concreteCallback( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg );
	
private:
	
	/// reference to the node handle
	ros::NodeHandle& nh;
	
	/// feedback manager
	action_feedback_manager fb;
	
	/// subscriber to the marker topic
	ros::Subscriber sub_marker;
	
	/// Navigation Client
	ros::ServiceClient cl_nav;
	
	/// last markers received
	visualization_msgs::MarkerArray last_msg_marker;
	
	/// waypoints 
	std::map<std::string, geometry_msgs::Pose> waypoints;
	
	/********************************************//**
	 *  
	 * \brief callback, subscriber to \ref SUBSCRIBER_MARKER
	 * 
	 * this callback only updates the markers. 
	 * 
	 * @param emptySignal empty 'request'
	 * 
	 * @see visualization_msgs/MarkerArray.msg [IN]
	 * 
	 ***********************************************/
	void cbk_marker( const visualization_msgs::MarkerArray::ConstPtr& pm );
	
	/********************************************//**
	 *  
	 * \brief write a pose message
	 * 
	 * @param x
	 * @param y
	 * @param z
	 * @param qx
	 * @param qy
	 * @param qx
	 * @param qw
	 * 
	 * @returns the pose ready to use
	 * 
	 ***********************************************/
	geometry_msgs::Pose make_pose( float x, float y, float z, float qx, float qy, float qz, float qw );
	
	/********************************************//**
	 *  
	 * \brief implementation of the action (move-to ?from ?to)
	 * 
	 ***********************************************/
	bool action_move_to( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg );
	
	/********************************************//**
	 *  
	 * \brief implementation of the action (move-to-center ?from)
	 * 
	 ***********************************************/
	bool action_move_to_center( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg );
};

}


#endif

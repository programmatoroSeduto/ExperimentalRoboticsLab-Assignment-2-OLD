
/********************************************//**
*  
* @file manipulator_near_marker.h
* 
* @brief implementation of the action (manipulator-near-marker ?wp )
* 
* @authors Francesco Ganci
* @version v1.0
* 
* ... more details
* 
***********************************************/

#ifndef __H_KCL_MANIPULATOR_NEAR_MARKER__
#define __H_KCL_MANIPULATOR_NEAR_MARKER__ "__H_MANIPULATOR_NEAR_MARKER__"

#include "ros/ros.h"
#include "knowledge_base_tools/robocluedo_kb_tools.h"
#include "robocluedo_rosplan_action_interface/RPActionInterface.h"

#define NODE_NAME "manipulator_near_marker"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#include "robocluedo_rosplan_interface_msgs/ActionFeedback.h"
#include "dispatch_actions/feedback_manager.h"

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

#include "robocluedo_rosplan_interface_msgs/ManipulationCommand.h"

#include <vector>
#include <unistd.h>
#include <cmath>

// markers update
#define TOPIC_MARKER "/visualization_marker"
#define Q_SZ 1

// manipulation unit
#define SERVICE_MANIP_UNIT "/robocluedo/manipulator_command"
#define TIMEOUT_MANIP_UNIT 60

namespace KCL_rosplan
{

/********************************************//**
 * 
 * \class RP_manipulator_near_marker
 * 
 * \brief implementation of the action manipulator_near_marker
 * 
 * ... more details
 * 
 ***********************************************/
class RP_manipulator_near_marker : public RPActionInterface, public robocluedo_kb_tools
{
public:
	
	/// empty constructor (don't use it!)
	RP_manipulator_near_marker( );
	
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
	 ***********************************************/
	RP_manipulator_near_marker( ros::NodeHandle &nh, bool debug_mode = KB_TOOLS_DEFAULT_DEBUG_MODE );
	
	/********************************************//**
	 *  
	 * \brief class destructor
	 * 
	 ***********************************************/
	~RP_manipulator_near_marker( );
	
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
	
	/// client to the manipulator unit
	ros::ServiceClient cl_manip_unit;
	
	/// subscriber to the marker topic
	ros::Subscriber sub_marker;
	
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
	 * \brief move the manipulator to a given target
	 * 
	 * @param target the point to reach with the manipulator
	 * 
	 * @returns true if the call succeeded and the manipulator moved 
	 * 	correctly
	 * 
	 ***********************************************/
	bool manipulator_at_pos( geometry_msgs::Point target );
	
	/********************************************//**
	 *  
	 * \brief move the manipulator to the home position
	 * 
	 * @returns true if the call succeeded and the manipulator moved 
	 * 	correctly
	 * 
	 ***********************************************/
	bool manipulator_home_position( );
};

}


#endif

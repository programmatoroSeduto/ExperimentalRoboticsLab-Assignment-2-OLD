
/********************************************//**
*  
* @file init_planning_system.h
* 
* @brief implementation of the action init_planning_system
* 
* @authors Francesco Ganci
* @version v1.0
* 
* ... more details
* 
***********************************************/

#ifndef __H_KCL_INIT_PLANNING_SYSTEM__
#define __H_KCL_INIT_PLANNING_SYSTEM__ "__H_INIT_PLANNING_SYSTEM__"

#define ROSPLAN_ACTION_NAME "init_planning_system"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( ROSPLAN_ACTION_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#include "ros/ros.h"
#include "knowledge_base_tools/robocluedo_kb_tools.h"
#include "rosplan_action_interface/RPActionInterface.h"

#include "diagnostic_msgs/KeyValue.h"
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

#include <vector>
#include <unistd.h>



namespace KCL_rosplan
{

/********************************************//**
 * 
 * \class RP_init_planning_system
 * 
 * \brief implementation of the action init_planning_system
 * 
 * ... more details
 * 
 ***********************************************/
class RP_init_planning_system : public RPActionInterface, public robocluedo_kb_tools
{
public:
	
	/********************************************//**
	 *  
	 * \brief class constructor
	 * 
	 * ... more details
	 * 
	 * @param nh (ros::NodeHandle) the handle of the node
	 * @param debug_mode verbose print or not?
	 * 
	 ***********************************************/
	RP_init_planning_system( ros::NodeHandle &nh, bool debug_mode = KB_TOOLS_DEFAULT_DEBUG_MODE );
	
	/********************************************//**
	 *  
	 * \brief class destructor
	 * 
	 ***********************************************/
	~RP_init_planning_system( );
	
	/********************************************//**
	 *  
	 * \brief listen to and process action_dispatch topic
	 * 
	 * ... more details
	 * 
	 * @param msg the action to execute
	 * 
	 * @returns useless, it always returns true
	 * 
	 ***********************************************/
	bool concreteCallback( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg );
	
private:
	
	/// reference to the node handle
	ros::NodeHandle& nh;
};

}


#endif

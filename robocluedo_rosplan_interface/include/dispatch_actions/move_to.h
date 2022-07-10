
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

#ifndef __H_KCL_INIT_PLANNING_SYSTEM__
#define __H_KCL_INIT_PLANNING_SYSTEM__ "__H_INIT_PLANNING_SYSTEM__"

#include "ros/ros.h"
#include "knowledge_base_tools/robocluedo_kb_tools.h"
#include "robocluedo_rosplan_action_interface/RPActionInterface.h"

#define NODE_NAME "move_to"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#include "diagnostic_msgs/KeyValue.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"

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
	 ***********************************************/
	RP_move_to( ros::NodeHandle &nh, bool debug_mode = KB_TOOLS_DEFAULT_DEBUG_MODE );
	
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
};

}


#endif

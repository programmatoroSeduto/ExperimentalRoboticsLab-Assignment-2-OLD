
/********************************************//**
*  
* @file init_planning_system.h
* 
* @brief implementation of the action (init-planning-system )
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

#include "robocluedo_rosplan_interface_msgs/ActionFeedback.h"
#include "dispatch_actions/feedback_manager.h"

#define NODE_NAME "init_planning_system"

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

#include "erl2/ErlOracle.h"
/*
int32 ID
string key
string value
*/

#include "erl2/Oracle.h"
/*
---
int32 ID
*/

#include <vector>
#include <unistd.h>

// Oracle service
#define SERVICE_ORACLE "/oracle_solution"
#define TIMEOUT_ORACLE 5



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
	
	/// empty constructor (don't use it!)
	RP_init_planning_system( );
	
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
	RP_init_planning_system( ros::NodeHandle &nh, bool debug_mode = KB_TOOLS_DEFAULT_DEBUG_MODE );
	
	/********************************************//**
	 *  
	 * \brief class destructor
	 * 
	 ***********************************************/
	~RP_init_planning_system( );
	
	/********************************************//**
	 *  
	 * \brief dispatch depending on the action called
	 * 
	 ***********************************************/
	bool concreteCallback( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg );
	
private:
	
	/// reference to the node handle
	ros::NodeHandle nh;
	
	/// oracle service client
	ros::ServiceClient cl_oracle;
	
	/// feedback manager
	action_feedback_manager fb;
	
	/********************************************//**
	 *  
	 * \brief classify the hypotheses
	 * 
	 * the system tracks the hypotheses into three categories: 
	 * <ul>
	 * <li><b>open</b> : there's no proof that it is incorrect, and some element is missing</li>
	 * <li><b>complete</b> : the hypotheses has its three elements</li>
	 * <li><b>discard</b> : at least one field has more than one option</li>
	 * </ul><br>
	 * 
	 * @param msg the action to execute
	 * 
	 * @returns false in case on unconsistenied in the ontology
	 * 
	 * @todo what about the UNKNOWN value?
	 * 
	 ***********************************************/
	bool classify_hypotheses( );
	
	/********************************************//**
	 *  
	 * \brief implementation of (init-planning-system )
	 * 
	 * here's the procedure: 
	 * <ol>
	 * <li> check the classification of the hypotheses, and send a failure
	 * feedback in case of unconsistency of the database </li>
	 * <li> check if the problem is still solvable, that is if not every
	 * hypothesis ID have been discarded </li>
	 * <li> check if the problem can be solved by exclusion, and in case
	 * send a feedback to the mission controller asking to replan and
	 * solve by exclusion </li>
	 * </ol>
	 * 
	 * @param msg the action to execute
	 * 
	 * @returns true if the action succeeded, false in case of problems
	 * 
	 * @note the solution by exclusion can be called when there's only one
	 * hypothesis which is complete OR simply active. 
	 * 
	 * @note the node can communicate with a node in charge to orchestrate
	 * the mission and, in particular, to take a decision when a replan action
	 * becomes necessary. 
	 * 
	 ***********************************************/
	bool action_init_planning_system( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg );
	
	/********************************************//**
	 *  
	 * \brief final action of the working cycle
	 * 
	 ***********************************************/
	bool action_end( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg );
	
};

}


#endif

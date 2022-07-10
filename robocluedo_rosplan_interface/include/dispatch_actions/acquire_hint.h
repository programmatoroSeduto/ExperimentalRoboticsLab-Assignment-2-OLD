
/********************************************//**
*  
* @file acquire_hint.h
* 
* @brief implementation of the action (acquire-hint ?wp)
* 
* @authors Francesco Ganci
* @version v1.0
* 
* ... more details
* 
***********************************************/

#ifndef __H_KCL_ACQUIRE_HINT__
#define __H_KCL_ACQUIRE_HINT__ "__H_ACQUIRE_HINT__"

#include "ros/ros.h"
#include "knowledge_base_tools/robocluedo_kb_tools.h"
#include "robocluedo_rosplan_action_interface/RPActionInterface.h"

#define NODE_NAME "acquire_hint"

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

/// default queue size
#define Q_SZ 1000

// hint topic
#define TOPIC_HINT "/oracle_hint"

namespace KCL_rosplan
{

/********************************************//**
 * 
 * \class RP_acquire_hint
 * 
 * \brief implementation of the action acquire_hint
 * 
 * ... more details
 * 
 ***********************************************/
class RP_acquire_hint : public RPActionInterface, public robocluedo_kb_tools
{
public:
	
	/// empty constructor (don't use it!)
	RP_acquire_hint( );
	
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
	RP_acquire_hint( ros::NodeHandle &nh, bool debug_mode = KB_TOOLS_DEFAULT_DEBUG_MODE );
	
	/********************************************//**
	 *  
	 * \brief class destructor
	 * 
	 ***********************************************/
	~RP_acquire_hint( );
	
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
	
	/// last hint received 
	erl2::ErlOracle last_hint;
	
	/// received a hint and not read before (turn off after reading the message)
	bool pending_messages;
	
	/********************************************//**
	 *  
	 * \brief subscriber listening for the hints from the Oracle
	 * 
	 * @param pm
	 * 		the hint from the Oracle, as message
	 * 
	 * @note we're making here strong assumptions about how the Oracle
	 * works. In particular, the oracle publishes only one message after
	 * the manipulator has reached the neighborhood of the marker. 
	 * 
	 ***********************************************/
	void cbk_hint( const erl2::ErlOracle::ConstPtr& pm );
	
	/********************************************//**
	 *  
	 * \brief check if the hint is valid or not
	 * 
	 * The oracle can send sometimes a not well-formed message, using one
	 * among these disturbances: 
	 * <ul>
	 * <li>missing fields</li>
	 * <li>value corresponding to a negative number</li>
	 * </ul>
	 * 
	 * @param hint 
	 * 		the hint message to check
	 * 
	 * @returns true if the message is not corrupted, false otherwise
	 * 
	 ***********************************************/
	bool is_valid_hint( erl2::ErlOracle hint );
};

}


#endif

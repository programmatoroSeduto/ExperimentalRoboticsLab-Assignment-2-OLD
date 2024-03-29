
/********************************************//**
*  
* @file sherlock_is_thinking.h
* 
* @brief implementation of the action (move-to ?from ?to )
* 
* @authors Francesco Ganci
* @version v1.0
* 
* ... more details
* 
***********************************************/

#ifndef __H_KCL_SHERLOCK_IS_THINKING__
#define __H_SHERLOCK_IS_THINKING__ "__H_SHERLOCK_IS_THINKING__"

#include "ros/ros.h"
#include "knowledge_base_tools/robocluedo_kb_tools.h"
#include "robocluedo_rosplan_action_interface/RPActionInterface.h"

#include "robocluedo_rosplan_interface_msgs/ActionFeedback.h"
#include "dispatch_actions/feedback_manager.h"

#define NODE_NAME "sherlock_is_thinking"

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

#include <vector>
#include <unistd.h>



namespace KCL_rosplan
{

/********************************************//**
 * 
 * \class RP_sherlock_is_thinking
 * 
 * \brief implementation of the action sherlock_is_thinking
 * 
 * ... more details
 * 
 ***********************************************/
class RP_sherlock_is_thinking : public RPActionInterface, public robocluedo_kb_tools
{
public:
	
	/// empty constructor (don't use it!)
	RP_sherlock_is_thinking( );
	
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
	RP_sherlock_is_thinking( ros::NodeHandle &nh, bool debug_mode = KB_TOOLS_DEFAULT_DEBUG_MODE );
	
	/********************************************//**
	 *  
	 * \brief class destructor
	 * 
	 ***********************************************/
	~RP_sherlock_is_thinking( );
	
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
	
	/********************************************//**
	 *  
	 * \brief update the ontology before starting with the reasoning
	 * 
	 * @returns true if there are not inconsistencies in the problem 
	 * formulation. 
	 * 
	 * @todo often we assume that every simple query works fine... but 
	 * is it really true? do we trust so much? 
	 * 
	 * @todo what about unknown classifications?
	 * 
	 ***********************************************/
	bool update_classes( );
};

}


#endif

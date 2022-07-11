
/********************************************//**
*  
* @file move_base_interface.h
* @brief ...
* 
* @authors Francesco Ganci
* @version v1.0
* 
***********************************************/

#define NODE_NAME "move_base_interface"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#include "ros/ros.h"
#include "tf/tf.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include <string>
#include <map>
#include <signal.h>

#include "geometry_msgs/PoseStamped.h"
/*
std_msgs/Header header
geometry_msgs/Pose pose
*/

#include "geometry_msgs/Pose.h"
/*
Point position
Quaternion orientation
*/

#include "geometry_msgs/Quaternion.h"
/*
float64 x, y, z, w
*/

#include "geometry_msgs/Point.h"
/*
float x, y, z
*/

#include "move_base_msgs/MoveBaseAction.h"
// move_base_msgs::MoveBaseGoal
/*
geometry_msgs/PoseStamped target_pose
*/
// move_base_msgs::MoveBaseFeedback
/*
geometry_msgs/PoseStamped base_position
*/
// move_base_msgs::MoveBaseResult
	// empty



// action client move base
#define ACTION_MOVE_BASE "move_base"
#define TIMEOUT_MOVE_BASE 100



class move_base_interface
{
public:

	// === BASE METHODS === //
	
	/// class constructor
	move_base_interface( );
	
	
	/// class destructor
	~move_base_interface( );
	
	
	
	// === DIRECT COMMUNICATION WITH MOVE_BASE === //
	
	/********************************************//**
	 *  
	 * \brief send a goal to the navigation system
	 *	
	 * 
	 ***********************************************/
	bool send_goal( 
		move_base_msgs::MoveBaseGoal goal, 
		bool wait = false, 
		ros::Duration d = ros::Duration( TIMEOUT_MOVE_BASE ) );
	
	
	/********************************************//**
	 *  
	 * \brief send a goal to the navigation system, onyl position
	 * 
	 ***********************************************/
	bool send_goal( 
		float x, float y, float z,
		bool wait = false, 
		ros::Duration d = ros::Duration( TIMEOUT_MOVE_BASE ) );
	
	
	/// cancel the last request
	bool cancel( );
	
	
	
	// === CALLBACKS === //
	
	/// Called once when the goal becomes active
	void cbk_active_move_base( );
	
	
	/// feedback subscription
	void cbk_feedback_move_base(
		const move_base_msgs::MoveBaseFeedbackConstPtr& feedback );
	

	/// Called once when the goal completes (not called after cancellation)
	void cbk_done_move_base(
		const actionlib::SimpleClientGoalState& state,
		const move_base_msgs::MoveBaseResultConstPtr& res );
	
	
	/// check activity flag
	bool is_running( );
	
	
	/// check activity flag
	bool is_idle( );
	
	
	/// action client status from the handle
	std::string get_state( );
	
private:
	
	/// the action client
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> actcl_move_base;
	
	/// the current goal
	move_base_msgs::MoveBaseGoal last_goal;
	
	/// activity flag
	bool running;
	
	/// idle flag
	bool idle;
};

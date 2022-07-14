
/********************************************//**
*  
* @file move_base_interface.cpp
* @brief ...
* 
* @authors Francesco Ganci
* @version v1.0
* 
***********************************************/

#include "move_base_interface/move_base_interface.h"



// === BASIC METHODS === //

// class constructor
move_base_interface::move_base_interface( ) : 
	actcl_move_base( "move_base", true ),
	running( false ), 
	idle( true )
{
	TLOG( "opening action client " << LOGSQUARE( ACTION_MOVE_BASE ) << " ... " );
	
	actcl_move_base.waitForServer( );
	if( !this->actcl_move_base.waitForServer( ros::Duration( TIMEOUT_MOVE_BASE ) ) )
	{
		TERR( "unable to connect to the action server (timeout " << TIMEOUT_MOVE_BASE << "s) " 
			<< "-- action " << LOGSQUARE( ACTION_MOVE_BASE ) << "\n"
			<< "\t " << (this->actcl_move_base.isServerConnected( ) ? " it seems not online " : " service online ") << "\n"
			<< "\t" << "STATUS: " << this->actcl_move_base.getState( ).toString( ) );
		
		return;
	}
	
	TLOG( "opening action client " << LOGSQUARE( ACTION_MOVE_BASE ) << " ... OK!" );
}


// class destructor
move_base_interface::~move_base_interface( )
{
	// ...
}




// === DIRECT COMMUNICATION WITH MOVE_BASE === //

// send a goal to the move_base action service
bool move_base_interface::send_goal( 
	move_base_msgs::MoveBaseGoal goal, 
	bool wait, 
	ros::Duration d )
{
	// call the action service 
	this->last_goal = goal;
	actcl_move_base.sendGoal( this->last_goal, 
		boost::bind( &move_base_interface::cbk_done_move_base, this, _1, _2 ),
		boost::bind( &move_base_interface::cbk_active_move_base, this ),
		boost::bind( &move_base_interface::cbk_feedback_move_base, this, _1 )
		);
	
	// wait until the end of the action or the timeout
	if( wait )
	{
		if( !actcl_move_base.waitForResult( d ) )
		{
			TERR( "action client for " << LOGSQUARE( ACTION_MOVE_BASE ) << "TIMEOUT EXPIRED " );
			actcl_move_base.cancelAllGoals( );
			
			// ...
			
			return false;
		}
	}
	
	// goal reached
	return true;
}


// simplification for the position only
bool move_base_interface::send_goal( 
	float x, 
	float y, 
	float z,
	std::string frame_id = "map",
	bool wait, 
	ros::Duration d )
{
	// prepare the goal
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = frame_id;
	goal.target_pose.pose.position.x = x;
	goal.target_pose.pose.position.y = y;
	goal.target_pose.pose.position.z = z;
	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = 0;
	goal.target_pose.pose.orientation.w = 1.0;
	
	// send the goal as usual
	return this->send_goal( goal, wait, d );
}


// cancel the last request if any
bool move_base_interface::cancel( )
{
	if( this->running )
	{
		actcl_move_base.cancelAllGoals( );
		
		this->idle = true;
		this->running = false;
		
		return true;
	}
	
	// idle! nothing to cancel
	return false;
}




// === CALLBACKS === //

// active callback
void move_base_interface::cbk_active_move_base( )
{
	this->idle = false;
	this->running = true;
	
	TLOG( "move_base ACTIVE" );
}


// feedback subscription
void move_base_interface::cbk_feedback_move_base(
	const move_base_msgs::MoveBaseFeedbackConstPtr& feedback )
{	
	/*
	float x = feedback->base_position.pose.position.x;
	float y = feedback->base_position.pose.position.y;
	float z = feedback->base_position.pose.position.z;
	
	TLOG( "move_base FEEDBACK : (" << x << ", " << y << ", " << z << ")" );
	*/
}


// result callback
void move_base_interface::cbk_done_move_base(
	const actionlib::SimpleClientGoalState& state,
	const move_base_msgs::MoveBaseResultConstPtr& res )
{
	this->idle = true;
	this->running = false;
	
	TLOG( "move_base STOP" );
	TLOG( "Finished in state " << state.toString( ) );
}




// === CHECKS === //

// check activity flag
bool move_base_interface::is_running( ) 
	{ return this->running; }


// check activity flag
bool move_base_interface::is_idle( ) 
	{ return this->idle; }


// action client status from the handle
std::string move_base_interface::get_state( ) 
	{ return actcl_move_base.getState( ).toString( ); }

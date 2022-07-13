/********************************************//**
*  
* @file manipulation_unit.cpp
* @brief mission manager bridge between the robocluedo ROSPlan framework 
* 	and the manipulation controller.
* 
* @authors Francesco Ganci
* @version v1.0 
* 
***********************************************/


#define NODE_NAME "manipualtion_unit"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#include "ros/ros.h"

#include <string>
#include <map>
#include <signal.h>

/********************************************//**
 *  
 * \class nove_manipulation_unit
 * 
 * \brief bridge between robocluedo ROSPlan interface and the manipulation
 * 	controller
 * 
 * this node has a simple structure: it implements a service in communication
 * with the manipulation controller, customized for what the ROSPlan 
 * system has to do, that is the action of going neat to the marher, and 
 * the other action of returning in the home position.
 * 
 * @note in future, someone maybe wants to add functionalities to ROSPlan
 * robocluedo interface: that's why this bridge is located in its node, and
 * not in only one node. 
 * 
 ***********************************************/
class node_manipulation_unit
{
public:
	
	/********************************************//**
	 *  
	 * \brief node constructor
	 * 
	 * the constructor opens the services and sets up the ROS communication
	 * interface. 
	 * 
	 ***********************************************/
	node_manipulation_unit( )
	{
		// ...
	}
	
	/// spin function: just swait for shutdown
	void spin( )
	{
		// simple spin
		ros::waitForShutdowm( );
	}
	
	/// @todo manipulation service? move the end effector
	
private:
	
	// ROS node handle
    ros::NodeHandle nh;
	
	// ...
};


void shut_msg( int sig )
{
	TLOG( "stopping... " );
	ros::shutdown( );
}


int main( int argc, char* argv[] )
{
	ros::init( argc, argv, NODE_NAME, ros::init_options::NoSigintHandler );
	signal( SIGINT, shut_msg );
	ros::AsyncSpinner spinner( 4 );
	spinner.start( );
	ros::NodeHandle nh;
	
	TLOG( "starting ... " );
	
	// TODO define here services and topics ...
	
	TLOG( "ready" );
	
	// TODO the functionality of the node ... 
	( ???( ) ).spin( );
	
	return 0;
}

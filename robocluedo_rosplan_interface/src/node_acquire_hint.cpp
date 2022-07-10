
/********************************************//**
*  
* @file node_acquire_hint.cpp
* 
* @brief implementation of the action (acquire-hint ?wp)
* 
* @authors Francesco Ganci
* @version v1.0
* 
* @see acquire_hint.h the header file
* 
***********************************************/

#include "ros/ros.h"
#include "dispatch_actions/acquire_hint.h"

#define NODE_NAME "node_acquire_hint"

#ifndef __DEBUG_MACROS__
	#define __DEBUG_MACROS__

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#endif

#include <string>
#include <map>
#include <signal.h>




/********************************************//**
 *  
 * \brief shutdown message
 * 
 ***********************************************/
void shut_msg( int sig )
{
	TLOG( "stopping... " );
	ros::shutdown( );
}




/********************************************//**
 *  
 * \brief ROS node main - node_acquire_hint
 * 
 * ... more details
 * 
 ***********************************************/
int main(int argc, char **argv) 
{
	ros::init(argc, argv, NODE_NAME, ros::init_options::AnonymousName);
	signal(SIGINT, shut_msg);
	ros::NodeHandle nh("~");
	
	// create PDDL action publisher
	TLOG( "starting ... " );
	KCL_rosplan::RP_acquire_hint ac( nh );
	
	// run the node
	TLOG( "ready" );
	ac.runActionInterface( );
	
	return 0;
}

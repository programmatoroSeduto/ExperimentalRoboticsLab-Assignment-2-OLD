
/********************************************//**
*  
* @file manipulator_near_marker.cpp
* 
* @brief implementation of the action (manipulator-near-marker ?wp )
* 
* @authors Francesco Ganci
* @version v1.0
* 
* @see manipulator_near_marker.h the header file
* 
***********************************************/

#include "ros/ros.h"
#include "dispatch_actions/manipulator_near_marker.h"

#define NODE_NAME "node_manipulator_near_marker"

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
 * \brief ROS node main - node_move_to
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
	KCL_rosplan::RP_manipulator_near_marker ac( nh );
	
	// run the node
	TLOG( "ready" );
	ac.runActionInterface( );
	
	return 0;
}

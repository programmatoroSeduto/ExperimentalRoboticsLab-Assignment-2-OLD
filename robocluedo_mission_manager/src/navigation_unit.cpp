/********************************************//**
*  
* @file navigation_unit.cpp
* @brief bridge between the navigation system and the robocluedo ROSPlan
* 	component
* 
* @authors Francesco Ganci
* @version v1.0 
* 
***********************************************/


#define NODE_NAME "navigation_unit"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#include "ros/ros.h"

#include <string>
#include <map>
#include <signal.h>

// navigation command service
#include "robocluedo_rosplan_interface/NavigationCommand.h"
#define SERVICE_NAV "/robocluedo/navigation_command"
ros::ServiceServer *srv_nav;

/********************************************//**
 *  
 * \class node_navigation_unit
 * 
 * \brief bridge between robocluedo ROSPlan interface and the navigation
 * 	controller
 * 
 * @note in future, someone maybe wants to add functionalities to ROSPlan
 * robocluedo interface: that's why this bridge is located in its node, and
 * not in only one node. 
 * 
 ***********************************************/
class node_navigation_unit
{
public:
	
	/// node constructor
	node_navigation_unit( )
	{
		// ...
	}
	
	/// spin function: just swait for shutdown
	void spin( )
	{
		// simple spin
		ros::waitForShutdowm( );
	}
	
	/********************************************//**
	 *  
	 * \brief implementation of service \ref SERVICE_NAV
	 * 
	 * @param request ...description
	 * @param response ...description
	 * 
	 * @see NavigationCommand.srv
	 * 
	 ***********************************************/
	 bool cbk_nav( 
		robocluedo_rosplan_interface::NavigationCommand::Request& req, 
		robocluedo_rosplan_interface::NavigationCommand::Response& res )
	{
		/// @todo implement me!
		
		return true;
	}
	
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
	
	node_navigation_unit node;
	
	OUTLOG( "Advertising service " << LOGSQUARE( SERVICE_NAV ) << "..." );
	ros::ServiceServer tsrv_nav = nh.advertiseService( SERVICE_NAV, &node_navigation_unit::cbk_nav, &node );
	srv_nav = &tsrv_nav;
	OUTLOG( "Advertising service " << LOGSQUARE( SERVICE_NAV ) << "... OK" );
	
	/// @todo service with the navigation system?
	
	TLOG( "ready" );
	
	node.spin( );
	
	return 0;
}

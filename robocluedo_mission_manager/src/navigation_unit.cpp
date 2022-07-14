/********************************************//**
*  
* @file navigation_unit.cpp
* 
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

// rosplan navigation command service
#include "robocluedo_rosplan_interface_msgs/NavigationCommand.h"
#define SERVICE_NAV "/robocluedo/navigation_command"
ros::ServiceServer *srv_nav;

// navigation controller client
#include "robocluedo_movement_controller_msgs/GoToPoint.h"
#define SERVICE_NAV_CONTROLLER "/go_to_point"
#define TIMEOUT_NAV_CONTROLLER 5
ros::ServiceClient *cl_nav_controller;

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
 * @note referring to the architectural approach of this project, a bridge
 * node such as this one allows to keep separated the two different 
 * components robocluedo_rosplan_interface and robocluedo_movement_controller.
 * Otherwise, each component should have imported messages from another
 * component, making the distinction more complex to manage. In general,
 * rospan should work without knowing how the movement is managed, and which
 * application managed that part of the system. 
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
	
	/// spin function: just wait for shutdown
	void spin( )
	{
		// simple spin
		ros::waitForShutdown( );
	}
	
	/********************************************//**
	 *  
	 * \brief implementation of service \ref SERVICE_NAV
	 * 
	 * a service which calls a client. The unit simply waits until the
	 * robot has reached the final position. The unit can also deal with
	 * some metrics useful to ensure to not get stuck in the service
	 * call. 
	 * 
	 * @param request ...description
	 * @param response ...description
	 * 
	 * @see NavigationCommand.srv
	 * 
	 ***********************************************/
	 bool cbk_nav( 
		robocluedo_rosplan_interface_msgs::NavigationCommand::Request& req, 
		robocluedo_rosplan_interface_msgs::NavigationCommand::Response& res )
	{
		/// @todo implement me!
		
		return true;
	}
	
private:
	
	/// ROS node handle
    ros::NodeHandle nh;
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
	
	// rosplan navigation
	TLOG( "Advertising service " << LOGSQUARE( SERVICE_NAV ) << "..." );
	ros::ServiceServer tsrv_nav = nh.advertiseService( SERVICE_NAV, &node_navigation_unit::cbk_nav, &node );
	srv_nav = &tsrv_nav;
	TLOG( "Advertising service " << LOGSQUARE( SERVICE_NAV ) << "... OK" );
	
	// navigation controller
	TLOG( "Opening client " << LOGSQUARE( SERVICE_NAV_CONTROLLER ) << "..." );
	ros::ServiceClient tcl_nav_controller = nh.serviceClient<robocluedo_movement_controller_msgs::GoToPoint>( SERVICE_NAV_CONTROLLER );
	if( !tcl_nav_controller.waitForExistence( ros::Duration( TIMEOUT_NAV_CONTROLLER ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_NAV_CONTROLLER << "s) " );
		return 0;
	}
	cl_nav_controller = &tcl_nav_controller;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_NAV_CONTROLLER ) << "... OK" );
	
	TLOG( "ready" );
	
	node.spin( );
	
	return 0;
}

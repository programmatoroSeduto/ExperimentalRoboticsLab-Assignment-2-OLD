/********************************************//**
*  
* @file manipulation_unit.cpp
* 
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

// rosplan manipulation command
#include "robocluedo_rosplan_interface_msgs/ManipulationCommand.h"
#define SERVICE_ROBOPLAN_MANIP "/robocluedo_manipulator_command"
ros::ServiceServer *srv_roboplan_manip;

// manipulation controller
#include "robocluedo_movement_controller_msgs/TipPosition.h"
#define SERVICE_MANIP "/tip_pos"
#define TIMEOUT_MANIP 5

// client definition
ros::ServiceClient *cl_manip;

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
	
	/// node constructor
	node_manipulation_unit( )
	{
		// ...
	}
	
	/// spin function: just swait for shutdown
	void spin( )
	{
		// simple spin
		ros::waitForShutdown( );
	}
	
	/********************************************//**
	 *  
	 * \brief service implementation for the rosplan interface
	 * 
	 * the service calls a client to perform the manipulation, which is 
	 * limited to the simple movement of the end effector in this case. 
	 * 
	 * @param req
	 * @param res
	 * 
	 * @returns always true
	 * 
	 ***********************************************/
	bool cbk_roboplan_manip( 
		robocluedo_rosplan_interface_msgs::ManipulationCommand::Request& req, 
		robocluedo_rosplan_interface_msgs::ManipulationCommand::Response& res )
	{
		/// @todo implementation
		
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
	
	node_manipulation_unit node;
	
	// rosplan manipulation service
	TLOG( "Advertising service " << LOGSQUARE( SERVICE_ROBOPLAN_MANIP  ) << "..." );
	ros::ServiceServer tsrv_roboplan_manip = nh.advertiseService( SERVICE_ROBOPLAN_MANIP, &node_manipulation_unit::cbk_roboplan_manip, &node );
	srv_roboplan_manip = &tsrv_roboplan_manip;
	TLOG( "Advertising service " << LOGSQUARE( SERVICE_ROBOPLAN_MANIP  ) << "... ok" );
	
	// manipulation controller
	TLOG( "Opening client " << LOGSQUARE( SERVICE_MANIP ) << "..." );
	ros::ServiceClient tcl_manip = nh.serviceClient<robocluedo_movement_controller_msgs::TipPosition>( SERVICE_MANIP );
	if( !tcl_manip.waitForExistence( ros::Duration( TIMEOUT_MANIP ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_MANIP << "s) " );
		return 0;
	}
	cl_manip = &tcl_manip;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_MANIP ) << "... OK" );
	
	TLOG( "ready" );
	
	( node ).spin( );
	
	return 0;
}

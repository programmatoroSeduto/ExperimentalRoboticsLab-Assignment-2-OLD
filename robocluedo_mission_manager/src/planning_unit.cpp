
/********************************************//**
*  
* @file planning_unit.cpp
* 
* @brief the main node of the Mission Manager
* 
* @authors Francesco Ganci
* @version v1.0
* 
***********************************************/


#define NODE_NAME ???

#ifndef __DEBUG_MACROS__
	#define __DEBUG_MACROS__

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#endif

#include "ros/ros.h"

#include <string>
#include <map>
#include <signal.h>

// rosplan pipeline manager
#include "robocluedo_rosplan_interface_msgs/RosplanPipelineManagerService.h"
#define SERVICE_ROBOPLAN "/robocluedo/rosplan_handle"
#define TIMEOUT_ROBOPLAN 5
ros::ServiceClient *cl_roboplan;


/********************************************//**
 *  
 * \class node_planning_unit
 * 
 * \brief interaction with robocluedo ROSPlan interface, main node
 * 
 * the planning unit is the main part of the architecture: it initalizes 
 * ROSPlan and starts the mission, assuming that the other nodes of this
 * pachage have beed correctly started. 
 * 
 * the main role of this node is to manage the workflow of the system by
 * replanning commands. This node simply continues to replan many times 
 * until either the mission is accomplished or it is impossible to reach 
 * the goal due to some error. 
 * 
 * @note the mission manager is made up of three different components: the
 * planning unit (interaction with ROSplan), manipulation unit (interaction
 * with MoveIt throught the manipulation controller) and navigation 
 * controller (interaction with MoveBase and the nav stack through the
 * navigation controller). They are three independent units, but the 
 * "head" of the system is the planning unit. 
 * 
 * @note this kind of structure of the package allows to easily add
 * other functionalities with some little changes. For instance, let's
 * say to want to add a sensing functionality. With this approach, it is
 * needed just to add a "sensing unit" able to integrate robocluedo
 * rosplan interface with the sensing unit. 
 * 
 ***********************************************/
class node_planning_unit
{
public:
	
	/// node constructor
	node_planning_unit( )
	{
		// ...
	}
	
	/********************************************//**
	 *  
	 * \brief planning unti working cycle
	 * 
	 * after initialized the system, the planning unit start working in
	 * loop: first, it calls the robocluedo rosplan interface through
	 * it service (in particular the pipeline manager). When the service
	 * returns, it checks the flags of the message in order to understand
	 * what the outcome of the plan was; if the goal has been reached, 
	 * the planning unit has nothing to do, and can be closed; otherwise,
	 * it tries to understand what'shappened and issues a replan command.
	 * 
	 * it works int his way until either the goal has been reached, or
	 * the mystery is unsolvable. 
	 * 
	 * 
	 ***********************************************/
	void spin(  )
	{
		/// @todo 
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
	
	TLOG( "Opening client " << LOGSQUARE( SERVICE_ROBOPLAN ) << "..." );
	ros::ServiceClient tcl_roboplan = nh.serviceClient<roobocluedo_rosplan_interface_msgs/RosplanPipelineManagerService>( SERVICE_ROBOPLAN );
	if( !tcl_roboplan.waitForExistence( ros::Duration( TIMEOUT_ROBOPLAN ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_ROBOPLAN << "s) " );
		return 0;
	}
	cl_roboplan = &tcl_roboplan;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_ROBOPLAN ) << "... OK" );
	
	TLOG( "ready" );
	
	( node_planning_unit( ) ).spin( ); 
	
	return 0;
}

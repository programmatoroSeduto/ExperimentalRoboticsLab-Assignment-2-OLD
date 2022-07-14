
/********************************************//**
*  
* @file navigation_controller.cpp
* 
* @brief implementation of the robot navigaton capability
* 
* @authors Francesco Ganci
* @version v1.0
* 
***********************************************/


#define NODE_NAME "navigation_controller"

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

#include "robocluedo_movement_controller_msgs/OdomData.h"
#include "robocluedo_movement_controller_msgs/LocalisationSwitch.h"
#include "robocluedo_movement_controller_msgs/GoToPoint.h"

/// infos from the localisation system
#define TOPIC_ODOM "/loc/odom_data"
#define Q_SZ_ODOM 1
ros::Subscriber *sub_odom;

/// localisation service status
#define SERVICE_LOC_STATUS "/loc/odom"
#define TIMEOUT_LOC_STATUS 5
ros::ServiceClient *cl_loc_status;

/// service navigation controller
#define SERVICE_NAV "/go_to_point"
ros::ServiceServer *srv_nav;

/********************************************//**
 *  
 * \brief node class of the navigation controller
 * 
 * job of the navigation controller is to interact with the navigation
 * stack in a more convenient way. it mainly implements a service
 * which accepts a target to reach less than a threshold. 
 * 
 ***********************************************/
class node_navigation_controller
{
public:
	
	/// node constructor
	node_navigation_controller( )
	{
		// ...
	}
	
	// main functionality of the class
	void spin( int n_loops_per_sec = 1 )
	{
		// TODO main functionality (even a simple spin)
		
		/*
		ros::spin( )
		*/
		
		/*
		ros::Rate freq( n_loops_per_sec );
		while( ros::ok( ) )
		{
			// ... do whatever you want
			
			// spin and wait
			ros::spin_once( );
			freq.sleep( );
		}
		*/
	}
	
	/********************************************//**
	 *  
	 * \brief odometry data subscription
	 * 
	 * the function reads the data from the localisation system, only if 
	 * the request has been sent. 
	 * 
	 ***********************************************/
	void cbk_odom( const robocluedo_movement_controller_msgs::OdomData::ConstPtr& pm )
	{
		if( this->read_from_odom_data )
		{
			/// @todo implementation
		}
	}
	
	/********************************************//**
	 *  
	 * \brief implementation of service \ref SERVICE_NAV
	 * 
	 * use the navigation stack to reach a given position and orientation.
	 * 
	 * @note specific for 2D navigation
	 * 
	 * @param req
	 * @param res 
	 * 
	 * @see file.srv
	 * 
	 ***********************************************/
	bool cbk_nav( 
		robocluedo_movement_controller_msgs::GoToPoint::Request& req, 
		robocluedo_movement_controller_msgs::GoToPoint::Response& res )
	{
		/// @todo implementation
		
		return true;
	}
	
private:
	
	/// ROS node handle
    ros::NodeHandle nh;
	
	/// read or not from odometry
	bool read_from_odom_data = false;
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
	
	node_navigation_controller node;
	
	// odometry data
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_ODOM ) << "..." );
	ros::Subscriber tsub_odom = nh.subscribe( TOPIC_ODOM, Q_SZ_ODOM, &node_navigation_controller::cbk_odom, &node );
	sub_odom = &tsub_odom;
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_ODOM ) << "... OK" );
	
	// localisation unit status
	TLOG( "Opening client " << LOGSQUARE( SERVICE_LOC_STATUS ) << "..." );
	ros::ServiceClient tcl_loc_status = nh.serviceClient<robocluedo_movement_controller_msgs::LocalisationSwitch>( SERVICE_LOC_STATUS );
	if( !tcl_loc_status.waitForExistence( ros::Duration( TIMEOUT_LOC_STATUS ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_LOC_STATUS << "s) " );
		return 0;
	}
	cl_loc_status = &tcl_loc_status;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_LOC_STATUS ) << "... OK" );
	
	// navigation system service
	TLOG( "Advertising service " << LOGSQUARE( SERVICE_NAV  ) << "..." );
	ros::ServiceServer tsrv_nav = nh.advertiseService( SERVICE_NAV, &node_navigation_controller::cbk_nav, &node );
	srv_nav = &tsrv_nav;
	TLOG( "Advertising service " << LOGSQUARE( SERVICE_NAV  ) << "... OK" );
	
	TLOG( "ready" );
	
	node.spin( );
	ros::waitForShutdown( );
	
	return 0;
}

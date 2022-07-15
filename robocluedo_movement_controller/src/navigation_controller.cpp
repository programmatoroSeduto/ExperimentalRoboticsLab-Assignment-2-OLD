
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

#include "ros/ros.h"
#include "move_base_interface/move_base_interface.h"

#define NODE_NAME "navigation_controller"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#include <string>
#include <map>
#include <signal.h>

#include "robocluedo_movement_controller_msgs/OdomData.h"
#include "robocluedo_movement_controller_msgs/LocalisationSwitch.h"
#include "robocluedo_movement_controller_msgs/GoToPoint.h"

#define DEFAULT_POSITION_THRESHOLD 0.1
#define DEFAULT_ORIENTATION_THRESHOLD 0.01
#define DEFAULT_LIMIT_COUNTER 30

/// infos from the localisation system
#define TOPIC_ODOM "/loc/odom"
#define Q_SZ_ODOM 1
ros::Subscriber *sub_odom;

/// localisation service status
#define SERVICE_LOC_STATUS "/loc/switch"
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
class node_navigation_controller : move_base_interface
{
public:
	
	/// node constructor
	node_navigation_controller( ):
		move_base_interface( )
	{
		// ...
	}
	
	/// simple spin
	void spin( int n_loops_per_sec = 1 )
	{
		ros::waitForShutdown( );
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
		// reach a position
		if( req.ask_position )
			res.position_success = this->move_to_point( req.target_position, req.threshold_position );
		
		/*
		if( res.position_success && req.ask_orientation )
			res.orientation.success = this
		*/
		
		return true;
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
			this->dist = pm->distance;
			TLOG( "received distance=" << pm->distance );
		}
		else
		{
			this->dist = 66666666.f;
			TLOG( "missing distance" );
		}
	}
	
private:
	
	/// ROS node handle
    ros::NodeHandle nh;
	
	/// read or not from odometry
	bool read_from_odom_data = false;
	
	/// distance from the target
	float dist = 66666666.f;
	
	/********************************************//**
	 *  
	 * \brief call movebase and move to a given target
	 * 
	 * @param tg the target
	 * @param threshold the maximum error allowed
	 * @param limit_counter a counter, incremented 1 by 1, used to track
	 * 	the time spent by move_base to reach the target; the node works
	 * 	with a frequency of 1 distance checking per second. 
	 * 
	 * @returns true if the movement succeeded, false otherwise
	 * 
	 ***********************************************/
	bool move_to_point( geometry_msgs::Point tg, 
		float threshold = DEFAULT_POSITION_THRESHOLD,
		int limit_counter = DEFAULT_LIMIT_COUNTER )
	{
		// check for a not valid threshold
		if( threshold < 0 )
			threshold = DEFAULT_POSITION_THRESHOLD;
		
		// activate the localisation system
		robocluedo_movement_controller_msgs::LocalisationSwitch sw;
		sw.request.new_status = true;
		sw.request.compute_distance = true;
		sw.request.pdist = tg;
		
		if( !cl_loc_status->call( sw ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_LOC_STATUS ) 
				<< (!cl_loc_status->exists( ) ? " -- it seems not opened" : "") );
			
			this->read_from_odom_data = false;
			return false;
		}
		
		if( !sw.response.success || !sw.response.active )
			return false;
		
		this->read_from_odom_data = true;
		
		// send the command to the nav stak
		this->send_goal( false, tg.x, tg.y, 0.0 );
		
		// wait for the distance to become zero
		ros::Rate r( 1 );
		bool reached = false;
		int counter = 0;
		float last_dist = 0.f;
		while( true )
		{
			// check the status of the action
			std::string status = this->get_state_str( );
			// PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
			if( status != "ACTIVE" )
			{
				// check success
				if( status == "SUCCEEDED" )
				{
					reached = true;
					TLOG( "target REACHED -- from move_base SUCCESS" );
					
					break;
				}
				else if ( status == "PENDING" )
				{
					/// @todo what is better to do given this situation?
					TLOG( "move_base status is PENDING" );
				}
				else
				{
					// failure
					reached = false;
					
					TLOG( "target NOT REACHED -- from move_base " << status );
					
					break;
				}
			}
			
			TLOG( "distance=" << this->dist );
			
			// check for the distance
			if( this->dist <= threshold )
			{
				this->cancel( );
				
				last_dist = this->dist;
				reached = true;
				
				TLOG( "target REACHED" );
				break;
			}
			else 
			{			
				if( counter < limit_counter )
					++counter;
				else
				{
					this->cancel( );
					last_dist = this->dist;
					
					TLOG( "target NO REACHED -- timeout expired" );
					break;
				}
			}
			
			// sleep
			r.sleep( );
			ros::spinOnce( );
		}
		
		// turn off the localisation system
		sw.request.new_status = false;
		if( !cl_loc_status->call( sw ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_LOC_STATUS ) 
				<< (!cl_loc_status->exists( ) ? " -- it seems not opened" : "") );
		}
		this->read_from_odom_data = false;
		
		return reached;
	}
	
	
	/********************************************//**
	 *  
	 * \brief orient the front of the robot of a given angle
	 * 
	 * @param tg the target
	 * @param threshold the maximum error allowed
	 * 
	 * @returns true if the movement succeeded, false otherwise
	 * 
	 ***********************************************/
	/*
	bool rotate_to_angle( float angle, 
		float threshold = DEFAULT_ORIENTATION_THRESHOLD )
	{
		// ...
		
		return true;
	}
	*/
	
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
	
	return 0;
}

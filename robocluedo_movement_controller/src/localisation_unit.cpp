
/********************************************//**
*  
* @file localisation_unit.cpp
* 
* @brief utilities and service for localizing the robot
* 
* @authors Francesco Ganci
* @version v1.0
* 
***********************************************/


#define NODE_NAME "localisation_unit"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#include "ros/ros.h"

#include <string>
#include <map>
#include <signal.h>

#include "nav_msgs/Odometry.h"
/*
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
*/

#include "robocluedo_movement_controller_msgs/LocalisationSwtich.h"

#include "robocluedo_movement_controller_msgs/OdomData.h"

// activity switch
#define SERVICE_SWITCH "/loc/switch"
ros::ServiceServer *srv_switch;

// localization infos publisher
#define TOPIC_ODOR "/loc/odom"
#define Q_SZ_ODOR 10
ros::Publisher *pub_odor;

// odom topic
#define TOPIC_ODOM "/odom"
#define Q_SZ_ODOM 1
ros::Subscriber *sub_odom;

/********************************************//**
 *  
 * \class node_localisation_unit
 * 
 * \brief the localisation unit
 * 
 * the navigation controller uses the functionalities provided by this node
 * for checking the progress towards a given goal position, for example. 
 * 
 * in general, the localisation unti provides functionalities to localise
 * the robot. in this case, the implementation is quite straightforward
 * because it is enough to read from the '/odom' topic and interpret the
 * data. 
 * 
 * noteworthy that this node provides a 2D localisation. 
 * 
 * the node opens two types of channes: services (for starting and 
 * stopping the communication) and one topic (to send the position). the 
 * idea is that another node should require the publication of a 
 * stream of messages.
 * 
 * @todo add the capability of the node to handle multiple requests: for
 * instance, one node could require the opening of one topic with a name
 * given by the caller. a single publisher could become here a vector of
 * publishers. 
 * 
 ***********************************************/

class node_localisation_unit
{
public:
	
	/// node constructor
	node_localisation_unit(  )
	{
		// ...
	}
	
	/********************************************//**
	 *  
	 * \brief position publisher imlementation
	 * 
	 * the node
	 * 
	 * @param param ... description
	 * 
	 * @returns ...description
	 * 
	 ***********************************************/
	void spin( int n_loops_per_sec = 1 )
	{
		/// @todo publisher!
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
	 * \brief implementation of service \ref SERVICE_SWITCH
	 * 
	 * ... more details
	 * 
	 * @param request ...description
	 * @param response ...description
	 * 
	 * @returns always true
	 * 
	 * @see file.srv
	 * 
	 ***********************************************/
	bool cbk_switch( 
		robocluedo_movement_controller::LocalisationSwitch::Request& req, 
		robocluedo_movement_controller::LocalisationSwitch::Response& res )
	{
		/// @todo implementation
		
		return true;
	}
	
	/********************************************//**
	 *  
	 * \brief odometry callback
	 * 
	 * @param param ... description
	 * 
	 * @returns ...description
	 * 
	 ***********************************************/
	void cbk_odom( const geometry_msgs::Odometry::ConstPtr& od )
	{
		/// @todo listener to topic /odom
	}
	
private:
	
	/// ROS node handle
    ros::NodeHandle nh;
	
	/// activity status of the node (default: off)
	bool active = false;
	
	/// distance required
	bool compute_distance = false;
	
	/// distance vector required
	bool compute_distance_vector = false;
	
	/// the target, if set
	geometry_msgs::Point target;
	
	/// the last position from the odometry
	geometry_msgs::Point current_pos;
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
	
	node_localisation_unit node;
	
	// switch service
	OUTLOG( "Advertising service " << LOGSQUARE( SERVICE_SWITCH  ) << "..." );
	ros::ServiceServer tsrv_switch = nh.advertiseService( SERVICE_SWITCH, &node_localisation_unit::cbk_switch, &node );
	srv_switch = &tsrv_switch;
	OUTLOG( "Advertising service " << LOGSQUARE( SERVICE_SWITCH  ) << "... OK" );
	
	// odometry publisher
	TLOG( "Creating publisher " << LOGSQUARE( TOPIC_ODOR ) << "..." );
	ros::Publisher tpub_odor = nh.advertise<robocluedo_movement_controller::OdomData>( TOPIC_ODOR, Q_SZ_ODOR );
	pub_odor = &tpub_odor;
	TLOG( "Creating publisher " << LOGSQUARE( TOPIC_ODOR ) << "..." );
	
	// odometry message from gazebo
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_ODOM ) << "..." );
	ros::Subscriber tsub_odom = nh.subscribe( TOPIC_ODOM, Q_SZ_ODOM, &node_localisation_unit::cbk_odom, &node );
	sub_odom = &tsub_odom;
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_ODOM ) << "... OK " );
	
	TLOG( "ready" );
	
	node.spin( ); 
	ros::waitForShutdown( );
	
	return 0;
}

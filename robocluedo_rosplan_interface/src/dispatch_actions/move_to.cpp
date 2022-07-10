
/********************************************//**
*  
* @file move_to.cpp
* 
* @brief implementation of the action (move-to ?from ?to )
* 
* @authors Francesco Ganci
* @version v1.0
* 
* @see move_to.h the header file
* 
***********************************************/


#include "dispatch_actions/move_to.h"



namespace KCL_rosplan
{

// === BASE METHODS === //

// the class constructor
RP_move_to::RP_move_to( ros::NodeHandle& nh, bool debug_mode ) : 
	RPActionInterface( ),
	robocluedo_kb_tools( debug_mode ),
	nh( nh )
{
	// setup manually the waypoints (https://quaternions.online)
	this->waypoints["center"] = this->make_pose( 0, 0, 0, 0, 0, 0, 1 );
	this->waypoints["wp1"] = this->make_pose( -2.75, 0, 0, 0, 0, 1, 0 );
	this->waypoints["wp2"] = this->make_pose( 2.75, 0, 0, 0, 0, 0, 1 );
	this->waypoints["wp3"] = this->make_pose( 0, -2.75, 0, 0, 0, -0.707, 0.707 );
	this->waypoints["wp4"] = this->make_pose( 0, 2.75, 0, 0, 0, 0.707, 0.707 );
	
	// topic for markers
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_MARKER ) << "..." );
	this->sub_marker = nh.subscribe( TOPIC_MARKER, Q_SZ, &RP_move_to::cbk_marker, this );
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_MARKER ) << "... OK" );
}


// class destructor
RP_move_to::~RP_move_to( )
{
	// ...
}




// === CONCRETE CALLBACK === //

// the callback
bool RP_move_to::concreteCallback( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
{
	// parameters
	auto params = this->keyvalue2map( msg->parameters );
	std::string from = params["from"];
	std::string to = params["to"];
	
	TLOG( "(move_to from=" << params["from"] << " to=" << params["to"] << ") CALLED" );
		
	/// @todo send the command to the navigation system and wait
	TWARN( "(TODO) sending position to the navigation system" );
	
	return true;
}




// === PRIVATE METHODS === //

// update the markers (one shot)
void RP_move_to::cbk_marker( const visualization_msgs::MarkerArray::ConstPtr& pm )
{
	// update the z component for each pose
	this->waypoints["wp1"].position.z = pm->markers[0].pose.position.z;
	this->waypoints["wp2"].position.z = pm->markers[1].pose.position.z;
	this->waypoints["wp3"].position.z = pm->markers[2].pose.position.z;
	this->waypoints["wp4"].position.z = pm->markers[3].pose.position.z;
	
	// shut down the subscriber 
	this->sub_marker.shutdown( );
	
	TLOG( "(move-to ) RECEIVED MARKERS" );
}

// write a pose
geometry_msgs::Pose RP_move_to::make_pose( float x, float y, float z, float qx, float qy, float qz, float qw )
{
	geometry_msgs::Pose p;
	
	// position
	p.position.x = x;
	p.position.y = y;
	p.position.z = z;
	
	// quaternion
	p.orientation.x = qx;
	p.orientation.y = qy;
	p.orientation.z = qz;
	p.orientation.w = qw;
	
	return p;
}

}

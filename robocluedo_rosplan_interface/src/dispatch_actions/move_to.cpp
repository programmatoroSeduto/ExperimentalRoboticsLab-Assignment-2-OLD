
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
	/*
	this->waypoints["wp1"] = this->make_pose( -2.75, 0, 0, 0, 0, 1, 0 );
	this->waypoints["wp2"] = this->make_pose( 2.75, 0, 0, 0, 0, 0, 1 );
	this->waypoints["wp3"] = this->make_pose( 0, -2.75, 0, 0, 0, -0.707, 0.707 );
	this->waypoints["wp4"] = this->make_pose( 0, 2.75, 0, 0, 0, 0.707, 0.707 );
	*/
	
	// topic for markers
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_MARKER ) << "..." );
	this->sub_marker = nh.subscribe( TOPIC_MARKER, Q_SZ, &RP_move_to::cbk_marker, this );
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_MARKER ) << "... OK" );

	// navigation client
	TLOG( "Opening client " << LOGSQUARE( SERVICE_NAV ) << "..." );
	this->cl_nav = nh.serviceClient<robocluedo_rosplan_interface_msgs::NavigationCommand>( SERVICE_NAV );
	if( !this->cl_nav.waitForExistence( ros::Duration( TIMEOUT_NAV ) ) )
	{
		TERR( "unable to contact the server " << SERVICE_NAV << " - timeout expired (" << TIMEOUT_NAV << "s) " );
		return;
	}
	TLOG( "Opening client " << LOGSQUARE( SERVICE_NAV ) << "... OK" );
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
	std::string action_name = msg->name;
	
	if( action_name == "move-to" )
	{
		fb.action_name = "move-to";
		return this->action_move_to( msg );
	}
	else
	{
		fb.action_name = "move-to-center";
		return this->action_move_to_center( msg );
	}
}




// === PRIVATE METHODS === //

// update the markers (one shot)
void RP_move_to::cbk_marker( const visualization_msgs::MarkerArray::ConstPtr& pm )
{
	// update the markers positions
	this->waypoints["wp1"] = pm->markers[0].pose;
	this->waypoints["wp2"] = pm->markers[1].pose;
	this->waypoints["wp3"] = pm->markers[2].pose;
	this->waypoints["wp4"] = pm->markers[3].pose;
	
	// shut down the subscriber 
	this->sub_marker.shutdown( );
	
	TLOG( "(move-to ) RECEIVED MARKERS" );
	
	for( int i=0; i < 4; ++i )
		TLOG( "\tMARKER no." << i << "(" << pm->markers[i].pose.position.x << ", " << pm->markers[i].pose.position.y << ")" );
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


// implementation of the action (move-to ?from ?to)
bool RP_move_to::action_move_to( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
{
	auto params = this->keyvalue2map( msg->parameters );
	std::string from = params["from"];
	std::string to = params["to"];
	
	TLOG( "(move-to from=" << params["from"] << " to=" << params["to"] << ") CALLED" );
		
	//send the command to the navigation system and wait
	robocluedo_rosplan_interface_msgs::NavigationCommand cmd;
	cmd.request.target = this->waypoints[ to ].position;
	cmd.request.target.x = 0.9 * cmd.request.target.x;
	cmd.request.target.y = 0.9 * cmd.request.target.y;
	cmd.request.look_to_marker = true;
	cmd.request.marker = this->waypoints[ to ].position;
	
	bool call_res = true;
	
	if( !this->cl_nav.call( cmd ) ) 
	{ 
		TERR( "unable to make a service request -- failed calling service " 
			<< LOGSQUARE( SERVICE_NAV ) 
			<< (!this->cl_nav.exists( ) ? " -- it seems not opened" : "") );
		
		call_res = false;
	}
	
	call_res = call_res && cmd.response.success;
	
	if( !call_res ) // in case of hw failure ...
	{
		fb.fb_hw_failure( msg->parameters, true, "navigation system failure" );
		return false;
	}
	
	return true;
}


// implementation of the action (move-to-center ?from)
bool RP_move_to::action_move_to_center( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
{
	auto params = this->keyvalue2map( msg->parameters );
	std::string from = params["from"];
	// std::string to = params["to"];
	
	TLOG( "(move-to-center from=" << params["from"] << ") CALLED" );
		
	//send the command to the navigation system and wait
	robocluedo_rosplan_interface_msgs::NavigationCommand cmd;
	cmd.request.target = this->waypoints["center"].position;
	cmd.request.look_to_marker = false;
	
	bool call_res = true;
	
	if( !this->cl_nav.call( cmd ) ) 
	{ 
		TERR( "unable to make a service request -- failed calling service " 
			<< LOGSQUARE( SERVICE_NAV ) 
			<< (!this->cl_nav.exists( ) ? " -- it seems not opened" : "") );
		
		call_res = false;
	}
	
	call_res = call_res && cmd.response.success;
	
	if( !call_res ) // in case of hw failure ...
	{
		fb.fb_hw_failure( msg->parameters, true, "navigation system failure" );
		return false;
	}
	
	return true;
}

}

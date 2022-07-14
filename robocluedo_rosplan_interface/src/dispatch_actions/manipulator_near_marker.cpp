
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


#include "dispatch_actions/manipulator_near_marker.h"



namespace KCL_rosplan
{

// === BASE METHODS === //

// the class constructor
RP_manipulator_near_marker::RP_manipulator_near_marker( ros::NodeHandle& nh, bool debug_mode ) : 
	RPActionInterface( ),
	robocluedo_kb_tools( debug_mode ),
	nh( nh )
{
	// one-shot topic for markers
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_MARKER ) << "..." );
	this->sub_marker = nh.subscribe( TOPIC_MARKER, Q_SZ, &RP_manipulator_near_marker::cbk_marker, this );
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_MARKER ) << "... OK" );

	// manipulation unit client
	TLOG( "Opening client " << LOGSQUARE( SERVICE_MANIP_UNIT ) << "..." );
	this->cl_manip_unit = nh.serviceClient<robocluedo_rosplan_interface_msgs::ManipulationCommand>( SERVICE_MANIP_UNIT );
	if( this->cl_manip_unit.waitForExistence( ros::Duration( TIMEOUT_MANIP_UNIT ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_MANIP_UNIT << "s) " );
		return;
	}
	TLOG( "Opening client " << LOGSQUARE( SERVICE_MANIP_UNIT ) << "... OK" );
}


// class destructor
RP_manipulator_near_marker::~RP_manipulator_near_marker( )
{
	// ...
}




// === CONCRETE CALLBACK === //

// the callback
bool RP_manipulator_near_marker::concreteCallback( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
{
	// read the parameters
	auto params = this->keyvalue2map( msg->parameters );
	std::string wp = params["wp"];
	
	if( debug_mode )
		TLOG( "(" << msg->name << " wp=" << wp << ") CALLED" );
	
	// feedback action name
	fb.action_name = msg->name;
	
	/// @todo send the point to reach to the manipulation servce
	bool outcome = true;
	if( msg->name == "manipulator-near-marker" )
	{
		// TWARN( "(TODO) put the manipulator near to the marker wp=" << wp );
		// update outcome
		outcome = this->manipulator_at_pos( this->waypoints[wp].position );
	}
	else
	{
		// TWARN( "(TODO) put the manipulator far from the marker wp=" << wp );
		// update outcome
		outcome = this->manipulator_home_position( );
	}
	
	// feedback in case of hardware failure
	if( !outcome )
	{
		fb.fb_hw_failure( msg->parameters, false, "manipulation failure" );
		return false;
	}
	
	return true;
}




// === PRIVATE METHODS === //

// update the markers (one shot)
void RP_manipulator_near_marker::cbk_marker( const visualization_msgs::MarkerArray::ConstPtr& pm )
{
	// update the z component for each pose
	this->waypoints["wp1"] = pm->markers[0].pose;
	this->waypoints["wp2"] = pm->markers[1].pose;
	this->waypoints["wp3"] = pm->markers[2].pose;
	this->waypoints["wp4"] = pm->markers[3].pose;
	
	// shut down the subscriber 
	this->sub_marker.shutdown( );
	
	TLOG( "(move-to ) RECEIVED MARKERS" );
}


// move the manipulator to a given target
bool RP_manipulator_near_marker::manipulator_at_pos( geometry_msgs::Point target )
{
	// request
	robocluedo_rosplan_interface_msgs::ManipulationCommand cmd;
	cmd.request.home_position = false;
	cmd.request.target_position = target;
	cmd.request.max_error = 0.1;
	
	// service call
	if( !this->cl_manip_unit.call( cmd ) ) 
	{ 
		TERR( "unable to make a service request -- failed calling service " 
			<< LOGSQUARE( SERVICE_MANIP_UNIT ) 
			<< (!this->cl_manip_unit.exists( ) ? " -- it seems not opened" : "") );
		
		return false;
	}
	
	return cmd.response.success;
}


// move the manipulator to the home position
bool RP_manipulator_near_marker::manipulator_home_position( )
{
	// request
	robocluedo_rosplan_interface_msgs::ManipulationCommand cmd;
	cmd.request.home_position = true;
	// cmd.request.target_position = target;
	cmd.request.max_error = 0.1;
	
	// service call
	if( !this->cl_manip_unit.call( cmd ) ) 
	{ 
		TERR( "unable to make a service request -- failed calling service " 
			<< LOGSQUARE( SERVICE_MANIP_UNIT ) 
			<< (!this->cl_manip_unit.exists( ) ? " -- it seems not opened" : "") );
		
		return false;
	}
	
	return cmd.response.success;
}

}

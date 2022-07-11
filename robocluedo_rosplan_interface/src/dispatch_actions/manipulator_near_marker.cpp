
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
		TWARN( "(TODO) put the manipulator near to the marker wp=" << wp );
		// update outcome
	}
	else
	{
		TWARN( "(TODO) put the manipulator far from the marker wp=" << wp );
		// update outcome
	}
	
	/// @todo feedback in case of hardware failure
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

}

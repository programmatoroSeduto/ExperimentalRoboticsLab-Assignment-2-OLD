
/********************************************//**
*  
* @file manipulator_far_marker.cpp
* 
* @brief implementation of the action (manipulator-far-marker ?wp )
* 
* @authors Francesco Ganci
* @version v1.0
* 
* @see manipulator_far_marker.h the header file
* 
***********************************************/


#include "dispatch_actions/manipulator_far_marker.h"



namespace KCL_rosplan
{

// === BASE METHODS === //

// the class constructor
RP_manipulator_far_marker::RP_manipulator_far_marker( ros::NodeHandle& nh, bool debug_mode ) : 
	RPActionInterface( ),
	robocluedo_kb_tools( debug_mode ),
	nh( nh )
{
	// ...
}


// class destructor
RP_manipulator_far_marker::~RP_manipulator_far_marker( )
{
	// ...
}




// === CONCRETE CALLBACK === //

// the callback
bool RP_manipulator_far_marker::concreteCallback( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
{
	bool res = true; 
	
	/// @todo a method to read the arguments each received each time the callback is issued
	
	if( debug_mode )
		TLOG( "(manipulator_far_marker ) CALLED" );
		
	// ...
	
	return true;

}




// === PRIVATE METHODS === //

// ...

}


/********************************************//**
*  
* @file move_to_center.cpp
* 
* @brief implementation of the action (move-to ?from ?to )
* 
* @authors Francesco Ganci
* @version v1.0
* 
* @see move_to_center.h the header file
* 
***********************************************/


#include "dispatch_actions/move_to_center.h"



namespace KCL_rosplan
{

// === BASE METHODS === //

// the class constructor
RP_move_to_center::RP_move_to_center( ros::NodeHandle& nh, bool debug_mode ) : 
	RPActionInterface( ),
	robocluedo_kb_tools( debug_mode ),
	nh( nh )
{
	// ...
}


// class destructor
RP_move_to_center::~RP_move_to_center( )
{
	// ...
}




// === CONCRETE CALLBACK === //

// the callback
bool RP_move_to_center::concreteCallback( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
{
	bool res = true; 
	
	/// @todo a method to read the arguments each received each time the callback is issued
	
	if( debug_mode )
		TLOG( "(move_to_center ) CALLED" );
		
	// ...
	
	return true;

}




// === PRIVATE METHODS === //

// ...

}

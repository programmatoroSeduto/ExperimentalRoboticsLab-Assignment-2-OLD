
/********************************************//**
*  
* @file acquire_hint.cpp
* 
* @brief implementation of the action (acquire-hint ?wp)
* 
* @authors Francesco Ganci
* @version v1.0
* 
* @see acquire_hint.h the header file
* 
***********************************************/


#include "dispatch_actions/acquire_hint.h"



namespace KCL_rosplan
{

// === BASE METHODS === //

// the class constructor
RP_acquire_hint::RP_acquire_hint( ros::NodeHandle& nh, bool debug_mode ) : 
	RPActionInterface( ),
	robocluedo_kb_tools( debug_mode ),
	nh( nh )
{
	// ...
}


// class destructor
RP_acquire_hint::~RP_acquire_hint( )
{
	// ...
}




// === CONCRETE CALLBACK === //

// the callback
bool RP_acquire_hint::concreteCallback( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
{
	bool res = true; 
	
	/// @todo a method to read the arguments each received each time the callback is issued
	
	if( debug_mode )
		TLOG( "(acquire_hint ) CALLED" );
		
	// ...
	
	return true;

}




// === PRIVATE METHODS === //

// ...

}


/********************************************//**
*  
* @file sherlock_is_thinking.cpp
* 
* @brief implementation of the action (move-to ?from ?to )
* 
* @authors Francesco Ganci
* @version v1.0
* 
* @see sherlock_is_thinking.h the header file
* 
***********************************************/


#include "dispatch_actions/sherlock_is_thinking.h"



namespace KCL_rosplan
{

// === BASE METHODS === //

// the class constructor
RP_sherlock_is_thinking::RP_sherlock_is_thinking( ros::NodeHandle& nh, bool debug_mode ) : 
	RPActionInterface( ),
	robocluedo_kb_tools( debug_mode ),
	nh( nh )
{
	// ...
}


// class destructor
RP_sherlock_is_thinking::~RP_sherlock_is_thinking( )
{
	// ...
}




// === CONCRETE CALLBACK === //

// the callback
bool RP_sherlock_is_thinking::concreteCallback( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
{
	bool res = true; 
	
	/// @todo a method to read the arguments each received each time the callback is issued
	
	if( debug_mode )
		TLOG( "(sherlock_is_thinking ) CALLED" );
		
	// ...
	
	return true;

}




// === PRIVATE METHODS === //

// ...

}

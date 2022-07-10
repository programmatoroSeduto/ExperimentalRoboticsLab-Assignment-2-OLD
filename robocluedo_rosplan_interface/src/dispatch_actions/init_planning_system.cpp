
/********************************************//**
*  
* @file init_planning_system.cpp
* 
* @brief implementation of the action init_planning_system
* 
* @authors Francesco Ganci
* @version v1.0
* 
* @see init_planning_system.h the header file
* 
***********************************************/



namespace KCL_rosplan
{

// === BASE METHODS === //

// the class constructor
RP_init_planning_system::RP_init_planning_system( ros::NodeHandle& nh, bool debug_mode ) : 
	RPActionInterface( ),
	robocluedo_kb_tools( debug_mode ),
	nh( nh )
{
	// ...
}


// class destructor
RP_init_planning_system::~RP_init_planning_system( )
{
	// ...
}




// === CONCRETE CALLBACK === //

// the callback
bool RP_init_planning_system::concreteCallback( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
{
	// ...
	
	return true;
}

}

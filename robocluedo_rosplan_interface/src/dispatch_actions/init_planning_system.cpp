
/********************************************//**
*  
* @file init_planning_system.cpp
* 
* @brief implementation of the action (init-planning-system )
* 
* @authors Francesco Ganci
* @version v1.0
* 
* @see init_planning_system.h the header file
* 
***********************************************/


#include "dispatch_actions/init_planning_system.h"



namespace KCL_rosplan
{

// === BASE METHODS === //

// empty constructor
RP_init_planning_system::RP_init_planning_system( ) :
	RPActionInterface( ),
	robocluedo_kb_tools( false )
{
	// ...
}

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
	bool res = true; 
	
	if( debug_mode )
		TLOG( "(init_planning_system ) CALLED" );
	
	// hypothese classification
	res = this->classify_hypotheses( );
	if( !res ) 
	{
		/// @todo send a feedback to the mission control, not consistent
		TWARN( "(init_planning_system ) PLAN FAILED : inconsistent problem state" );
		
		// plan failed
		return false;
	}
	
	int num_ids = this->get_num_of_ids( );
	int num_open = this->get_open_ids( );
	int num_complete = this->get_complete_ids( );
	int num_discard = this->get_discard_ids( );
	
	// check if the problem is still solvable
	if( num_discard >= num_ids )
	{
		/// @todo send a feedback to the mission control, unsolvable
		TWARN( "(init_planning_system ) PLAN FAILED : not solvable" );
		
		// unsolvable
		return false;
	}
	
	// in particular, check if the problem is solvable by exclusion
	if( ((num_complete + num_open) == 1) && (num_discard == (num_ids - 1)) )
	{
		/// @todo send a feedback to the mission control, solve by exclusion
		TLOG( "(init_planning_system ) PLAN SOLVABLE : by exclusion" );
		
		// replanning required
		return false;
	}
	
	/// @todo sed a feedback to the mission contro, plan solvable in a common way
	TLOG( "(init_planning_system ) PLAN SOLVABLE : common way" );
	
	return true;

}




// === PRIVATE METHODS === //

// classify the hypotheses
bool RP_init_planning_system::classify_hypotheses( )
{
	hypothesis_class cls;
	int num_hyp = this->get_num_of_ids( );
	
	for( int i=1; i<=num_hyp; ++i )
	{
		// update the hypothesis
		this->update_hypothesis( i, cls );
		
		// check the new status of the hypothesis (also detect inconsistencies)
		if( (cls == hypothesis_class::UNCONSISTENT_NO_CLASS) || (cls == hypothesis_class::UNCONSISTENT_REDUNDANT) )
		{	
			// plan failed
			return false;
		}
	}
	
	return true;
}

}

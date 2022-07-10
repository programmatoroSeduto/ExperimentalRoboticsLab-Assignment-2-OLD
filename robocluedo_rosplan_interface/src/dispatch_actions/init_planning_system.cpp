
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
	bool res = true; 
	
	// hypothese classification
	this->classify_hypotheses( );
	if( !res ) 
	{
		/// @todo send a feedback to the mission control, not consistent
		
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
		
		// unsolvable
		return false
	}
	
	// in particular, check if the problem is solvable by exclusion
	if( ((num_complete + num_open) == 1) && (num_discard == (num_ids - 1)) )
	{
		/// @todo send a feedback to the mission control, solve by exclusion
		
		// replanning required
		return false;
	}
	
	return true;

}




// === PRIVATE METHODS === //

// classify the hypotheses
bool RP_init_planning_system::classify_hypotheses( )
{
	robocluedo_kb_tools::hypothesis_class cls;
	int num_hyp = this->get_num_of_ids( );
	
	for( int i=1; i<=num_hyp; ++i )
	{
		// update the hypothesis
		this>update_hypothesis( i, cls );
		
		// check the new status of the hypothesis (also detect inconsistencies)
		if( cls == robocluedo_kb_tools::hypothesis_class::UNCONSISTENT_NO_CLASS ||
			cls == robocluedo_kb_tools::hypothesis_class::UNCONSISTENT_REDUNDANT )
		{	
			// plan failed
			return false;
		}
	}
	
	return true;
}

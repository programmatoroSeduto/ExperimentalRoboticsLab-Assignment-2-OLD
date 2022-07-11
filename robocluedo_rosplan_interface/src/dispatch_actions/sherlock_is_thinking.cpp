
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
	if( debug_mode )
		TLOG( "(sherlock_is_thinking wp=" << msg->parameters[0].value << ") CALLED" );
		
	// update the ontology
	if( !this->update_classes( ) )
	{
		TLOG( "(sherlock_is_thinking w) PROBLEM NOT CONSISTENT! plan failed" );
		return false;
	}
	
	// current status of the ontology
	int n_ids  = this->get_num_of_ids( );
	int n_open = this->get_open_ids( );
	int n_comp = this->get_complete_ids( );
	int n_disc = this->get_discard_ids( );
	
	if( debug_mode )
	{
		TLOG( "current status of the ontology: \n" 
			<< "\t " << n_open << " open IDs \n"
			<< "\t " << n_comp << " complete IDs \n"
			<< "\t " << n_disc << " discarded IDs " );
	}
	
	if( (n_open + n_comp) == 0 )
	{
		TWARN( "NOT SOLVABLE, give up" );
		
		/// @todo feedback to the mission control manager, give up
		
		return false;
	}
	else if( ((n_comp + n_open) == 1) && (n_disc == (n_ids - 1)) )
	{
		TLOG( "(num_complete + num_open)=" << (n_comp + n_open) << " && " <<
			"(num_discard=" << n_disc << ") == " << (n_ids - 1) << " SOLVABLE BY EXCLUSION" );
			
		/// @todo feedback to the mission control manager
		
		return false;
	}
	
	// the mission can go on
	return true;

}




// === PRIVATE METHODS === //

// update the ontology
bool RP_sherlock_is_thinking::update_classes( )
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

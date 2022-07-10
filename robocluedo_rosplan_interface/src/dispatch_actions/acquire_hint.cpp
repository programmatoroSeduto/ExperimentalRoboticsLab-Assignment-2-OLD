
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
	nh( nh ),
	pending_messages( false )
{
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_HINT ) << "..." );
	this->sub_hint = nh.subscribe( TOPIC_HINT, Q_SZ, cbk_hint );
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_HINT ) << "... OK" );
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
	
	// just to be sure that the message has been received
	ros::spin_once( );
		
	// check if there's a pending message
	if( !this->pending_messages )
	{
		TWARN( "NO MESSAGES FROM THE ORACLE ... skipping to the next action of the plan" );
		return true;
	}
	
	// check the validity of the message
	if( !this->is_valid_hint( this->last_hint ) )
	{
		TLOG( "MALFORMED HINT FROM THE ORACLE ... skipping to the next action of the plan" );
		return true;
	}
	
	hypothesis_class cls;
	
	// store the hint in the knowledge base
	if( !this->add_hint( last_hint.ID, last_hint.key, last_hint.value ) )
	{
		TWARN( "KB SET FAILED! unable to store correctly the hint, PLAN ABORTED" );
		return false;
	}
	else if( !this->update_hypothesis( last_hint.ID, cls ) )
	{
		TWARN( "KB UPDATE FAILED! unable to update the hint, try later" );
	}
	
	// everything done
	return true;

}




// === PRIVATE METHODS === //

// subscriber listening for the hints from the Oracle
void RP_acquire_hint::cbk_hint( const erl2::ErlOracle::ConstPtr& pm )
{
	// store the hint into the class
	this->last_hint.ID = pm->ID;
	this->last_hint.key = pm->key;
	this->last_hint.value = pm->value;
	
	// pending messages!
	this->pending_messages = true;
}


// check if the hint is valid or not
bool RP_acquire_hint::is_valid_hint( erl2::ErlOracle hint )
{
	return (hint.ID >= 0) && (hint.key != "" ) && 
		(hint.key != "-1") && (hint.value != "") && (hint.value != "-1");
}

}

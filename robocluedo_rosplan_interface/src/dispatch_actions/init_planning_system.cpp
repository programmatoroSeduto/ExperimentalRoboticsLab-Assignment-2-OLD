
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
	/// @todo open the service with the Oracle
	/*
	TLOG( "Opening client " << LOGSQUARE( SERVICE_ORACLE ) << "..." );
	this->cl_oracle = nh.serviceClient<erl2::Oracle>( SERVICE_ORACLE );
	if( !this->cl_oracle.waitForExistence( ros::Duration( TIMEOUT_ORACLE ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_ORACLE << "s) " );
		return;
	}
	TLOG( "Opening client " << LOGSQUARE( SERVICE_ORACLE ) << "... OK" );
	*/
}

// the class constructor
RP_init_planning_system::RP_init_planning_system( ros::NodeHandle& nh, bool debug_mode ) : 
	RPActionInterface( ),
	robocluedo_kb_tools( debug_mode ),
	nh( nh )
{
	/// @todo open the service with the Oracle
	/*
	TLOG( "Opening client " << LOGSQUARE( SERVICE_ORACLE ) << "..." );
	this->cl_oracle = nh.serviceClient<erl2::Oracle>( SERVICE_ORACLE );
	if( !this->cl_oracle.waitForExistence( ros::Duration( TIMEOUT_ORACLE ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_ORACLE << "s) " );
		return;
	}
	TLOG( "Opening client " << LOGSQUARE( SERVICE_ORACLE ) << "... OK" );
	*/
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
	if( msg->name == "init-planning-system" )
	{
		this->fb.action_name = "init-planning-system";
		return this->action_init_planning_system( msg );
	}
	else if( msg->name == "who-killed-doctor-black-huh-q1" )
	{
		TLOG( "(who-killed-doctor-black-huh-q1 id=" << msg->parameters[0].value << ") CALLED" );
		this->fb.action_name = "who-killed-doctor-black-huh-q1";
		return this->action_end( msg );
	}
	else
	{
		TLOG( "(who-killed-doctor-black-huh-q2 id=" << msg->parameters[0].value << ") CALLED" );
		this->fb.action_name = "who-killed-doctor-black-huh-q2";	
		return this->action_end( msg );
	}	
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


// implementation of (init-planning-system )
bool RP_init_planning_system::action_init_planning_system( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
{
	bool res = true; 
	
	TLOG( "(init_planning_system ) CALLED" );
	
	// hypothese classification
	res = this->classify_hypotheses( );
	if( !res ) 
	{
		// send a feedback to the mission control, not consistent
		TWARN( "(init_planning_system ) PLAN FAILED : inconsistent problem state" );
		fb.fb_unconsistent( msg->parameters, false, 
			"(init_planning_system ) PLAN FAILED : inconsistent problem state" );
		
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
		// send a feedback to the mission control, unsolvable
		TWARN( "(init_planning_system ) PLAN FAILED : not solvable" );
		fb.fb_unsolvable( msg->parameters, 
			"(init_planning_system ) PLAN FAILED : not solvable" );
		
		// unsolvable
		return false;
	}
	
	// in particular, check if the problem is solvable by exclusion
	if( ((num_complete + num_open) == 1) && (num_discard == (num_ids - 1)) )
	{
		// send a feedback to the mission control, solve by exclusion
		TLOG( "(init_planning_system ) PLAN SOLVABLE : by exclusion" );
		fb.fb_solvable( msg->parameters, true, 
			"(init_planning_system ) PLAN SOLVABLE : by exclusion" );
		
		TLOG( "remaining-moves reset to 0 (solution by exclusion)" );
		std::map<std::string, std::string> m;
		this->set_fluent( "remaining-moves", m, 0 );
		
		// replanning required
		return false;
	}
	
	TLOG( "(init_planning_system ) PLAN SOLVABLE : common way" );
	return true;

}


// implementation of the actions (who-killed-doctor-black-huh-q1 ) and (who-killed-doctor-black-huh-q2 )
bool RP_init_planning_system::action_end( const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg )
{
	// type of reward choosen
	bool solution_max_reward = ( msg->name == "who-killed-doctor-black-huh-q2" );
	
	// current status of the ontology
	int n_ids  = this->get_num_of_ids( );
	int n_open = this->get_open_ids( );
	int n_comp = this->get_complete_ids( );
	int n_disc = this->get_discard_ids( );
	
	// check also for the solution by exclusion
	bool solution_by_exclusion = ((n_comp + n_open) == 1) && (n_disc == (n_ids - 1));
	if( solution_by_exclusion )
		TLOG( "trying to solve by exclusion" );
	
	// the ID contained in the solution
	int prop_id = atoi( msg->parameters[0].value.substr( 2, msg->parameters[0].value.length( ) ).c_str( ) );
	
	if( solution_max_reward || solution_by_exclusion )
	{
		// check the solution with the Oracle
		TLOG( "calling the Oracle ..." );
		
		erl2::Oracle sm;
		if( !this->cl_oracle.call( sm ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_ORACLE ) 
				<< (!this->cl_oracle.exists( ) ? " -- it seems not opened" : "") );
			
			return false;
		}
		
		TLOG( "from Oracle : " << sm.response.ID << " || from the robot : " << prop_id );
		if( sm.response.ID == prop_id )
		{
			TLOG( "SOLUTION FOUND! id=" << prop_id );
			return true;
		}
		else
		{
			// discard the ID 
			TLOG( "not the solution. discard" );
			
			this->discard_hypothesis( prop_id );
		}
	}
	
	// unacceptable solution -- REPLAN
	
	/// @todo a more clever way to assign the number of moves? 
	
	TLOG( "remaining-moves reset to 3" );
	std::map<std::string, std::string> m;
	this->set_fluent( "remaining-moves", m, 3 );
	
	// send a replanning feedback
	fb.fb_replan( msg->parameters );
	
	TLOG( "REPLAN" );
	return false;
}

}

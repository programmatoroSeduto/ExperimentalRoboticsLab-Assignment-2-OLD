
/********************************************//**
*  
* @file feedback_manager.h
* @brief abstraction for the feedback publication
* 
* @authors Francesco Ganci
* @version v1.0
*  
* @see feedback_manager
* 
***********************************************/

#include "dispatch_actions/feedback_manager.h"




// === BASE METHODS === //

// class constructor
action_feedback_manager::action_feedback_manager( std::string act_name ) : 
	action_name( act_name )
{
	// open the action feedback topic
	TLOG( "Creating publisher " << LOGSQUARE( TOPIC_ACTION_FEEDBACK ) << "..." );
	this->pub_action_feedback = nh.advertise<robocluedo_rosplan_interface_msgs::ActionFeedback>( TOPIC_ACTION_FEEDBACK, Q_SZ );
	TLOG( "Creating publisher " << LOGSQUARE( TOPIC_ACTION_FEEDBACK ) << "... OK" );
}


// class destructor
action_feedback_manager::~action_feedback_manager( )
{
	// ...
}




// === FEEDBACK SYSTEM === //

// knowledge base not in a consistent status
bool action_feedback_manager::fb_unconsistent( std::vector<diagnostic_msgs::KeyValue> parameters, bool goal, std::string details )
{
	// the message
	robocluedo_rosplan_interface_msgs::ActionFeedback msg = build_msg( 
		parameters,	/* action params */ 
		false,		/* problem_solvable */
		false,		/* by_exclusion */
		true,		/* kb_not_consistent */
		goal,		/* goal_achieved */
		false,		/* need_replan */
		false,		/* failure_nav_sys */
		false,		/* failure_manipulation */
		details		/* annotations about the error */
		);
	
	// publish the message 
	return this->pub( msg );
}


// problem solvable
bool action_feedback_manager::fb_solvable( std::vector<diagnostic_msgs::KeyValue> parameters, bool excl, std::string details  )
{
	// the message
	robocluedo_rosplan_interface_msgs::ActionFeedback msg = build_msg( 
		parameters,	/* action params */ 
		true,		/* problem_solvable */
		true,		/* by_exclusion */
		false,		/* kb_not_consistent */
		false,		/* goal_achieved */
		false,		/* need_replan */
		false,		/* failure_nav_sys */
		false,		/* failure_manipulation */
		details		/* annotations about the error */
		);
	
	// publish the message 
	return this->pub( msg );
}


// problem unsolvable
bool action_feedback_manager::fb_unsolvable( std::vector<diagnostic_msgs::KeyValue> parameters, std::string details  )
{
	// the message
	robocluedo_rosplan_interface_msgs::ActionFeedback msg = build_msg( 
		parameters,	/* action params */ 
		false,		/* problem_solvable */
		false,		/* by_exclusion */
		false,		/* kb_not_consistent */
		false,		/* goal_achieved */
		false,		/* need_replan */
		false,		/* failure_nav_sys */
		false,		/* failure_manipulation */
		details		/* annotations about the error */
		);
	
	// publish the message 
	return this->pub( msg );
}


// need for replan (other reasons)
bool action_feedback_manager::fb_replan( std::vector<diagnostic_msgs::KeyValue> parameters, std::string details  )
{
	// the message
	robocluedo_rosplan_interface_msgs::ActionFeedback msg = build_msg( 
		parameters,	/* action params */ 
		true,		/* problem_solvable */
		false,		/* by_exclusion */
		false,		/* kb_not_consistent */
		false,		/* goal_achieved */
		true,		/* need_replan */
		false,		/* failure_nav_sys */
		false,		/* failure_manipulation */
		details		/* annotations about the error */
		);
	
	// publish the message 
	return this->pub( msg );
}

// hardware failure
bool action_feedback_manager::fm_hw_failure( std::vector<diagnostic_msgs::KeyValue> parameters, bool navigation, std::string details )
{
	// the message
	robocluedo_rosplan_interface_msgs::ActionFeedback msg = build_msg( 
		parameters,		/* action params */ 
		true,			/* problem_solvable */
		false,			/* by_exclusion */
		false,			/* kb_not_consistent */
		false,			/* goal_achieved */
		true,			/* need_replan */
		navigation,		/* failure_nav_sys */
		!navigation,	/* failure_manipulation */
		details			/* annotations about the error */
		);
	
	// publish the message 
	return this->pub( msg );
}




// === PRIVATE METHODS === //

// build a message
robocluedo_rosplan_interface_msgs::ActionFeedback action_feedback_manager::build_msg( 
		std::vector<diagnostic_msgs::KeyValue> parameters,
		bool problem_solvable,
		bool by_exclusion,
		bool kb_not_consistent,
		bool goal_achieved,
		bool need_replan,
		bool failure_nav_sys,
		bool failure_manipulation,
		std::string details )
{
	if( this->action_name == ACTION_NAME_UNKNOWN )
		TWARN( "action name of the feedback manager is UNKNOWN" );
	
	robocluedo_rosplan_interface_msgs::ActionFeedback m;
	
	// header
	m.action_name = this->action_name;
	m.parameters = parameters;
	
	// feedback properties
	m.problem_solvable = problem_solvable;
	m.by_exclusion = by_exclusion;
	m.kb_not_consistent = kb_not_consistent;
	m.goal_achieved = goal_achieved;
	m.need_replan = need_replan;
	m.failure_nav_system = failure_nav_sys;
	m.failure_manipulation = failure_manipulation;
	
	return m;
}


// publish the feedback
bool action_feedback_manager::pub( robocluedo_rosplan_interface_msgs::ActionFeedback& msg )
{
	this->pub_action_feedback.publish( msg );
	return true;
}

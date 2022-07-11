
/********************************************//**
*  
* @file robocluedo_pipeline_manager.cpp
* @brief controller for the ROSPlan workflow
* 
* @authors Francesco Ganci
* @version v1.0
* 
***********************************************/

#define NODE_NAME "robocluedo_pipeline_manager"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#include "ros/ros.h"

#include <string>
#include <map>
#include <signal.h>

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"

#include "std_srvs/Empty.h" 
/*
---
*/

#include "rosplan_dispatch_msgs/DispatchService.h"
/*
---
bool success
bool goal_achieved
*/

#include "robocluedo_rosplan_interface_msgs/RosplanPipelineManagerService.h"
/*
# service 'RosplanPipelineManagerService'

## Request

## === operations

# run the problem instance, then plan and parse
bool load_and_plan
# run the dispatcher
bool execute_plan

# number of max loops given by the mission manager
bool use_max_moves
int32 max_moves

---

## Response

## === load_and_plan request
# each one is true when the service call succeeds
bool plan_loaded
bool solution_found
bool plan_parsed

## === plan execution request

# reserved for any technical problem
bool exec_success
# goal status
bool goal_achieved

## === plan execution statistics

# inconsistency
bool kb_not_consistent
# problem solvable (false: not solvable)
bool problem_solvable
# problem solvable by exclusion
bool by_exclusion

*/

#include "robocluedo_rosplan_interface_msgs/ActionFeedback.h"
/*
# message 'ActionFeedback'

## === about the action sending the feedback message

# feedback from action
string action_name
# with parameters
diagnostc_msgs/KeyValue[ ] parameters

## === plan statistics

# inconsistency
bool kb_not_consistent
# problem solvable (false: not solvable)
bool problem_solvable
# problem solvable by exclusion
bool by_exclusion
*/

#define Q_SZ 1000

// service to load the problem in rosplan
#define SERVICE_PROBLEM_GENERATION_SERVER "/rosplan_problem_interface/problem_generation_server"
#define TIMEOUT_PROBLEM_GENERATION_SERVER 5
ros::ServiceClient *cl_problem_generation_server;

// generate the plan with the planning interface
#define SERVICE_PLANNING_SERVER "/rosplan_planner_interface/planning_server"
#define TIMEOUT_PLANNING_SERVER 5
ros::ServiceClient *cl_planning_server;

// generate the plan with the planning interface
#define SERVICE_PARSE_PLAN "/rosplan_parsing_interface/parse_plan"
#define TIMEOUT_PARSE_PLAN 5
ros::ServiceClient *cl_parse_plan;

// dispatcher trigger
#define SERVICE_DISPATCH "/rosplan_plan_dispatcher/dispatch_plan"
#define TIMEOUT_DISPATCH 5
ros::ServiceClient *cl_dispatch;

// action feedback topic
#define TOPIC_ACTION_FEEDBACK "/robocluedo/action_feedback"
ros::Subscriber *sub_action_feedback;

// rosplan handle service
#define SERVICE_ROSPLAN_HANDLE "/robocluedo/rosplan_handle"
ros::ServiceServer *srv_rosplan_handle;

// KB update (fluents and predicates)
#define SERVICE_KB_UPDATE "/rosplan_knowledge_base/update"
#define TIMEOUT_KB_UPDATE 5




/********************************************//**
 *  
 * \class robocluedo_pipeline_manager
 * 
 * \brief service controller for the ROSplan workflow
 * 
 * This node provides a number of services a mission manager can use to
 * end the mission. 
 * 
 ***********************************************/
class robocluedo_pipeline_manager
{
public:
	
	/********************************************//**
	 *  
	 * \brief node class constructor for robocluedo_pipeline_manager
	 * 
	 * empty constructor
	 * 
	 ***********************************************/
	robocluedo_pipeline_manager(  ):
		pipeline_working( false )
	{
		// ...
	}
	
	/********************************************//**
	 *  
	 * \brief simple spin
	 * 
	 * in this case, the node contains only one call to ros::spin( ).
	 * 
	 ***********************************************/
	void spin( )
	{	
		ros::spin( )
	}



private:
	
	/// ROS node handle
    ros::NodeHandle nh;
    
    /// is the pipeline working?
    bool pipeline_working;
    
    /// last action feedback from the topic
    robocluedo_rosplan_interface_msgs::ActionFeedback last_feedback;
    
	/********************************************//**
	 *  
	 * \brief service to handle the workflow
	 * 
	 * @param request ...description
	 * @param response ...description
	 * 
	 * @returns true if the service call succeeded
	 * 
	 * @note important assumption to made upon the execution phase: the
	 * feedback message is received before the service has been released.
	 * 
	 ***********************************************/
	bool cbk_rosplan_handle( 
		robocluedo_rosplan_interface_msgs::RosplanPipelineManagerService::Request& req, 
		robocluedo_rosplan_interface_msgs::RosplanPipelineManagerService::Request& res )
	{	
		// load - solve - parse
		if( req.load_and_plan )
		{
			this->pipeline_working = true;
			
			// set the number of loops if required by the service
			if( req.use_max_moves )
				this->set_max_moves( req.max_moves ); /// @todo return val?
			
			// problem instance
			TLOG( "creating problem instance ... " );
			res.plan_loaded = this->pddl_load( );
			TLOG( "creating problem instance ... OK" );
			
			// solution of the problem
			TLOG( "calling the planner and solving ... " );
			res.solution_found = this->pddl_solve( ); /// @todo is it really solved?
			TLOG( "calling the planner and solving ... OK" );
			
			// plan parsing
			TLOG( "parsing the plan ... " );
			res.plan_parsed = this->pddl_parse_plan( );
			TLOG( "parsing the plan ... OK" );
			
			this->pipeline_working = false;
		}
		
		// plan execution
		if( req.execute_plan )
		{
			this->pipeline_working = true;
			
			// exec the plan
			TLOG( "executing plan ... " );
			rosplan_dispatch_msgs::DispatchService from_dispatcher;
			this->dispatch_plan( from_dispatcher );
			TLOG( "executing plan ... OK" );
			
			// wait, to be sure that the message has been received
			TLOG( "waiting ... " )
			(ros::Duration(1)).sleep( );
			ros::spin_once( ); /// @todo is it blocking forever? what about not received feedbacks?
			TLOG( "waiting ... OK" )
			
			res.exec_success = from_dispatcher.response.success;
			res.goal_achieved = from_dispatcher.response.goal_achieved;
			
			/// @todo feeddback
			
			this->pipeline_working = false;
		}
		
		// ...
		
		return true;
	}
	
	/********************************************//**
	 *  
	 * \brief listen for a feedback from the pipeline
	 * 
	 * ... more details
	 * 
	 ***********************************************/
	void cbk_action_feedback( const robocluedo_rosplan_interface_msgs::ActionFeedback::ConstPtr& msg )
	{
		// skip the message if the pipeline is not working actually
		if( pipeline_working )
		{
			// copy the message inside the last message
			this->last_feedback.action_name = msg->action_name;
			this->last_feedback.parameters = msg->parameters;
			this->last_feedback.kb_not_consistent = msg->kb_not_consistent;
			this->last_feedback.problem_solvable = msg->problem_solvable;
			this->last_feedback.by_exclusion = msg->by_exclusion;
		}
	}
	
	/********************************************//**
	 *  
	 * \brief load the PDDL model, run the problem instance node
	 * 
	 * @returns true if the service call succeeded
	 * 
	 ***********************************************/
	bool pddl_load( )
	{
		// load the service
		std_srvs::Empty sm;
		if( !cl_problem_generation_server->call( sm ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_PROBLEM_GENERATION_SERVER ) 
				<< (!cl_problem_generation_server->exists( ) ? " -- it seems not opened" : "") );
			return false;
		}
		
		return true;
	}
	
	/********************************************//**
	 *  
	 * \brief get the plan of the problem
	 * 
	 * @returns true if the service call succeeded
	 * 
	 ***********************************************/
	bool pddl_solve( )
	{
		std_srvs::Empty sm;
		if( !cl_planning_server->call( sm ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_PLANNING_SERVER ) 
				<< (!cl_planning_server->exists( ) ? " -- it seems not opened" : "") );
			return false;
		}
		
		return true;
	}
	
	/********************************************//**
	 *  
	 * \brief parse the plan
	 * 
	 * @returns true if the service call succeeded
	 * 
	 ***********************************************/
	void pddl_parse_plan( )
	{
		std_srvs::Empty sm;
		if( !cl_parse_plan->call( sm ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_PARSE_PLAN ) 
				<< (!cl_parse_plan->exists( ) ? " -- it seems not opened" : "") );
			return;
		}
		
		(ros::Duration(2)).sleep( );
	}
	
	/********************************************//**
	 *  
	 * \brief trigger the plan dispatch
	 * 
	 * @param return_log( rosplan_dispatch_msgs::DispatchService&, output )
	 * 	the message from the service, field used as a second return value
	 * 	from the function. 
	 * 
	 * @returns true if the service call succeeded
	 * 
	 ***********************************************/
	 bool dispatch_plan( rosplan_dispatch_msgs::DispatchService& return_loc )
	{
		rosplan_dispatch_msgs::DispatchService sm;
		if( !cl_dispatch->call( sm ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_DISPATCH ) 
				<< (!cl_dispatch->exists( ) ? " -- it seems not opened" : "") );
			
			sm.response.success = false;
			sm.response.goal_achieved = false;
		}
		
		return_loc = sm;
		return sm.response.success;
	}
	
	/********************************************//**
	 *  
	 * \brief set the fluent (remaining-moves )
	 * 
	 * the method uses a bare service call.
	 * 
	 * @param max_moves the new value of the fluent
	 * 
	 * @returns false if either the number is negative or the service is
	 * 	not reachable or the fluent doesn't exist (buggy scenario) or other 
	 * 	situation requiring a false return by the knowledge base, 
	 * 	true otherwise. 
	 * 
	 ***********************************************/
	bool set_max_moves( int max_moves )
	{
		// check the number
		if( max_moves < 0 )
		{
			TWARN( "(set_max_moves) no a valid moves value (given " << max_moves << ")" );
			return false;
		}
		
		// check if the service exists
		if( !ros::service::exists( SERVICE_KB_UPDATE ) )
		{
			TWARN( "(set_max_moves) unable to call the knowledge base - service '" << SERVICE_KB_UPDATE << "' doesn't exist" );
			return false;
		}
		
		// query
		rosplan_knowledge_msgs::KnowledgeUpdateService kbm;
		kbm.request.update_type = KB_ADD_KNOWLEDGE;
		kbm.request.knowledge.knowledge_type = KB_KTYPE_FLUENT;
		kbm.request.knowledge.attribute_name = "remaining-moves";
		kbm.request.knowledge.function_value = max_moves;
		
		// service call
		if( !ros::service::call( SERVICE_KB_UPDATE, sm ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_KB_UPDATE ) 
				<< (!ros::service::exists( SERVICE_KB_UPDATE )) ? " -- it seems not opened" : "") );
			return 0;
		}
		
		return kbm.response.success;
	}
};





void shut_msg( int sig )
{
	TLOG( "stopping... " );
	ros::shutdown( );
}


int main( int argc, char* argv[] )
{
	ros::init( argc, argv, NODE_NAME, ros::init_options::NoSigintHandler );
	signal( SIGINT, shut_msg );
	ros::NodeHandle nh;
	
	// create a async spinner
	ros::AsyncSpinner spinner( 3 );
	spinner.start( );
	
	TLOG( "starting ... " );
	
	// the class node
	robocluedo_pipeline_manager this_node;
	
	// client problem generation service
	TLOG( "Opening client " << LOGSQUARE( SERVICE_PROBLEM_GENERATION_SERVER ) << "..." );
	ros::ServiceClient tcl_problem_generation_server = nh.serviceClient<std_srvs::Empty>( SERVICE_PROBLEM_GENERATION_SERVER );
	if( !tcl_problem_generation_server.waitForExistence( ros::Duration( TIMEOUT_PROBLEM_GENERATION_SERVER ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_PROBLEM_GENERATION_SERVER << "s) " );
		return 0;
	}
	cl_problem_generation_server = &tcl_problem_generation_server;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_PROBLEM_GENERATION_SERVER ) << "... OK" );

	TLOG( "Opening client " << LOGSQUARE( SERVICE_PLANNING_SERVER ) << "..." );
	ros::ServiceClient tcl_planning_server = nh.serviceClient<std_srvs::Empty>( SERVICE_PLANNING_SERVER );
	if( !tcl_planning_server.waitForExistence( ros::Duration( TIMEOUT_PLANNING_SERVER ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_PLANNING_SERVER << "s) " );
		return 0;
	}
	cl_planning_server = &tcl_planning_server;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_PLANNING_SERVER ) << "... OK" );
	
	TLOG( "Opening client " << LOGSQUARE( SERVICE_PARSE_PLAN ) << "..." );
	ros::ServiceClient tcl_parse_plan = nh.serviceClient<std_srvs::Empty>( SERVICE_PARSE_PLAN );
	if( !tcl_parse_plan.waitForExistence( ros::Duration( TIMEOUT_PARSE_PLAN ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_PARSE_PLAN << "s) " );
		return 0;
	}
	cl_parse_plan = &tcl_parse_plan;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_PARSE_PLAN ) << "... OK" );
	
	TLOG( "Opening client " << LOGSQUARE( SERVICE_DISPATCH ) << "..." );
	ros::ServiceClient tcl_dispatch = nh.serviceClient<rosplan_dispatch_msgs::DispatchService>( SERVICE_DISPATCH );
	if( !tcl_dispatch.waitForExistence( ros::Duration( TIMEOUT_DISPATCH ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_DISPATCH << "s) " );
		return 0;
	}
	cl_dispatch = &tcl_dispatch;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_DISPATCH ) << "... OK" );
	
	// subscriber: feedback topic
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_ACTION_FEEDBACK ) << "..." );
	ros::Subscriber tsub_action_feedback = nh.subscribe( TOPIC_ACTION_FEEDBACK, Q_SZ, &robocluedo_pipeline_manager::cbk_action_feedback, &this_node );
	sub_action_feedback = &tsub_action_feedback;
	TLOG( "subscribing to the topic " << LOGSQUARE( TOPIC_ACTION_FEEDBACK ) << "... OK" );
	
	// service: rosplan handle
	OUTLOG( "Advertising service " << LOGSQUARE( SERVICE_ROSPLAN_HANDLE  ) << "..." );
	ros::ServiceServer tsrv_rosplan_handle = nh.advertiseService( SERVICE_ROSPLAN_HANDLE, &robocluedo_pipeline_manager::cbk_rosplan_handle, &this_node );
	srv_rosplan_handle = &tsrv_rosplan_handle;
	OUTLOG( "Advertising service " << LOGSQUARE( SERVICE_ROSPLAN_HANDLE ) << "... OK" );
	
	TLOG( "ready" );
	this_node.spin( );
	
	return 0;
}

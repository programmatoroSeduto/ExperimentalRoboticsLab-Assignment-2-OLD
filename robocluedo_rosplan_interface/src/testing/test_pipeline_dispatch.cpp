
#include <ros/ros.h>
#include <signal.h>


#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "diagnostic_msgs/KeyValue.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_dispatch_msgs/DispatchService.h"

#define NODE_NAME "test_pipeline_dispatch"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )


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


class test_load_and_run
{
public:
	
	// node constructor
	test_load_and_run( ) { }
	
	// load the PDDL model, solve and parse it
	void spin(  )
	{
		TLOG( "=== TESTING ROSPLAN PIPELINE ===" );
		
		TLOG( "loading model ... " );
		this->pddl_load( );
		TLOG( "loading model ... OK" );
		
		TLOG( "solving the problem ... " );
		this->pddl_solve( );
		TLOG( "solving the problem ... OK" );
		
		TLOG( "parsing ... " );
		this->pddl_parse_plan( );
		TLOG( "parsing ... OK" );
		
		TLOG( "running the plan ... " );
		this->dispatch_plan( );
		TLOG( "running the plan ... OK" );
	}
	
	// load the PDDL model, run the problem instance node
	void pddl_load( )
	{
		// load the service
		std_srvs::Empty sm;
		if( !cl_problem_generation_server->call( sm ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_PROBLEM_GENERATION_SERVER ) 
				<< (!cl_problem_generation_server->exists( ) ? " -- it seems not opened" : "") );
			return;
		}
		
		(ros::Duration(2)).sleep( );
	}
	
	// get the plan of the problem
	void pddl_solve( )
	{
		std_srvs::Empty sm;
		if( !cl_planning_server->call( sm ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_PLANNING_SERVER ) 
				<< (!cl_planning_server->exists( ) ? " -- it seems not opened" : "") );
			return;
		}
		
		(ros::Duration(2)).sleep( );
	}
	
	// parse the plan
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
	
	// trigger the plan dispatch
	void dispatch_plan( )
	{
		rosplan_dispatch_msgs::DispatchService sm;
		if( !cl_dispatch->call( sm ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_DISPATCH ) 
				<< (!cl_dispatch->exists( ) ? " -- it seems not opened" : "") );
			return;
		}
		
		(ros::Duration(2)).sleep( );
		
		if( sm.response.success )
			TLOG( "plan dispatch " );
		else
			TLOG( "plan dispatch returned .success=false" );
		
		if( sm.response.goal_achieved )
			TLOG( "plan dispatch GOAL_ACHIEVED" );
		else
			TLOG( "plan dispatch returned .goal_achieved=false" );
		
	}
	
	
protected:
	
	// ROS node handle
    ros::NodeHandle nh;
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
	
	TLOG( "starting ... " );
	
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
	
	TLOG( "ready" );
	
	( test_load_and_run( ) ).spin( );
	
	return 0;
}

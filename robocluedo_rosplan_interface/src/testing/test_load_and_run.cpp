
#include <ros/ros.h>
#include <signal.h>

#include "std_srvs/Empty.h" 
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "diagnostic_msgs/KeyValue.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"

#define NODE_NAME "test_load_and_run"

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

// service for updating the ontology
#define SERVICE_KB_UPDATE "/rosplan_knowledge_base/update"
#define TIMEOUT_KB_UPDATE 5
ros::ServiceClient *cl_kb_update;
#define KB_ADD_KNOWLEDGE 0
#define KB_DEL_KNOWLEDGE 2
#define KB_KTYPE_FLUENT 2
#define KB_KTYPE_PREDICATE 1

// fluent from kb
#define SERVICE_KB_GET_FLUENT "/rosplan_knowledge_base/state/functions"
#define TIMEOUT_KB_GET_FLUENT 5
ros::ServiceClient *cl_kb_get_fluent;

/*
// predicates from kb
#define SERVICE_KB_GET_PRED "/rosplan_knowledge_base/state/propositions"
#define TIMEOUT_KB_GET_PRED 5
ros::ServiceClient *cl_kb_get_pred;
*/

// query service of the kb
#define SERVICE_QUERY "/rosplan_knowledge_base/query_state"
#define TIMEOUT_QUERY 5
ros::ServiceClient *cl_query;



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
		
		
		
		TLOG( "=== TESTING FLUENTS ===" );
		
		TLOG( "setting fluent 'f-non-zero=4' ... " );
		{
			this->set_fluent( "f-non-zero", 4.0 );
		}
		TLOG( "setting fluent 'f-non-zero=4' ... OK" );
		
		TLOG( "reading value of 'f-non-zero' ... " );
		{
			float val = 0.0;
			bool rt = this->get_fluent( "f-non-zero", val );
			TLOG( "f-non-zero=" << val << (rt ? " returned TRUE" : " returned FALSE") );
		}
		TLOG( "reading value of 'f-non-zero' ... OK" );
		
		TLOG( "setting fluent 'f-non-zero=6' ... " );
		{
			this->set_fluent( "f-non-zero", 6 );
		}
		TLOG( "setting fluent 'f-non-zero=6' ... OK" );
		
		TLOG( "reading value of 'f-non-zero' ... " );
		{
			float val = 0.0;
			bool rt = this->get_fluent( "f-non-zero", val );
			TLOG( "f-non-zero=" << val << (rt ? " returned TRUE" : " returned FALSE") );
		}
		TLOG( "reading value of 'f-non-zero' ... OK" );
		
		
		
		TLOG( "=== TESTING PREDICATES WITH NO ARGS ===" );
		
		TLOG( "setting predicate '(signal-stop )=FALSE' ... " );
		{
			std::map<std::string, std::string> m;
			this->set_predicate( "signal-stop", m, false );
		}
		TLOG( "setting predicate '(signal-stop )=FALSE' ... OK" );
		
		TLOG( "reading value of '(signal-stop )' ... " );
		{
			std::map<std::string, std::string> m;
			bool res = this->get_predicate( "signal-stop", m );
			TLOG( "expected FALSE -- (signal-stop )=" << (res ? "TRUE" : "FALSE") );
		}
		TLOG( "reading value of '(signal-stop )' ... OK" );
		
		TLOG( "setting predicate '(signal-stop )=TRUE' ... " );
		{
			std::map<std::string, std::string> m;
			this->set_predicate( "signal-stop", m, true );
		}
		TLOG( "setting predicate '(signal-stop )=TRUE' ... OK" );
		
		TLOG( "reading value of '(signal-stop )' ... " );
		{
			std::map<std::string, std::string> m;
			bool res = this->get_predicate( "signal-stop", m );
			TLOG( "expected TRUE -- (signal-stop )=" << (res ? "TRUE" : "FALSE") );
		}
		TLOG( "reading value of '(signal-stop )' ... OK" );
		
		
		
		TLOG( "=== TESTING PREDICATES WITH ONE ARG ===" );
		
		TLOG( "setting predicate '(b-true b1)=FALSE' ... " );
		{
			std::map<std::string, std::string> m;
			m["b"] = "b1";
			this->set_predicate( "b-true", m, false );
		}
		TLOG( "setting predicate '(b-true b1)=FALSE' ... OK" );
		
		TLOG( "reading value of '(b-true b1)' ... " );
		{
			std::map<std::string, std::string> m;
			m["b"] = "b1";
			bool res = this->get_predicate( "b-true", m );
			TLOG( "expected FALSE -- (b-true b1)=" << (res ? "TRUE" : "FALSE") );
		}
		TLOG( "reading value of '(b-true b1)' ... OK" );
		
		TLOG( "setting predicate '(b-true b1)=TRUE' ... " );
		{
			std::map<std::string, std::string> m;
			m["b"] = "b1";
			this->set_predicate( "b-true", m, true );
		}
		TLOG( "setting predicate '(b-true b1)=TRUE' ... OK" );
		
		TLOG( "reading value of '(b-true b1)' ... " );
		{
			std::map<std::string, std::string> m;
			m["b"] = "b1";
			bool res = this->get_predicate( "b-true", m );
			TLOG( "expected TRUE -- (b-true b1)=" << (res ? "TRUE" : "FALSE") );
		}
		TLOG( "reading value of '(b-true b1)' ... OK" );
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
	}
	
	
protected:
	
	// ROS node handle
    ros::NodeHandle nh;
    
    // set a simple fluent
    bool set_fluent( const std::string fname, float fvalue )
    {
		// prepare command
		rosplan_knowledge_msgs::KnowledgeUpdateService kbm;
		
		kbm.request.update_type = KB_ADD_KNOWLEDGE;
		kbm.request.knowledge.knowledge_type = KB_KTYPE_FLUENT;
		kbm.request.knowledge.attribute_name = fname;
		kbm.request.knowledge.function_value = fvalue;
		
		// send the command
		if( !cl_kb_update->call( kbm ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_KB_UPDATE ) 
				<< (!cl_kb_update->exists( ) ? " -- it seems not opened" : "") );
			return false;
		}
		
		return kbm.response.success;
	}
	
	bool set_predicate( const std::string pname, std::map<std::string, std::string>& params, bool value=true )
	{
		// prepare command
		rosplan_knowledge_msgs::KnowledgeUpdateService kbm;
		
		kbm.request.update_type = ( value ? KB_ADD_KNOWLEDGE : KB_DEL_KNOWLEDGE );
		kbm.request.knowledge.knowledge_type = KB_KTYPE_PREDICATE;
		kbm.request.knowledge.attribute_name = pname;
		
		for ( auto it=params.begin( ) ; it!=params.end( ) ; ++it )
		{
			diagnostic_msgs::KeyValue kv;
			kv.key = it->first;
			kv.value = it->second;
			kbm.request.knowledge.values.push_back( kv );
		}
		
		// send command
		if( !cl_kb_update->call( kbm ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_KB_UPDATE ) 
				<< (!cl_kb_update->exists( ) ? " -- it seems not opened" : "") );
			return false;
		}
		
		// return success
		return kbm.response.success;
	}
	
	// rval: return here the fluent value
	bool get_fluent( const std::string fname, float& rval )
	{
		// prepare command
		rosplan_knowledge_msgs::GetAttributeService kbm;
		kbm.request.predicate_name = fname;
		
		// call the service
		if( !cl_kb_get_fluent->call( kbm ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_KB_GET_FLUENT ) 
				<< (!cl_kb_get_fluent->exists( ) ? " -- it seems not opened" : "") );
			return false;
		}
		
		// return the value
		rval = kbm.response.attributes[0].function_value;
		return true;
	}
	
	bool get_predicate( const std::string pname, std::map<std::string, std::string>& params )
	{
		// query message
		rosplan_knowledge_msgs::KnowledgeQueryService query;
		rosplan_knowledge_msgs::KnowledgeItem kbm;
		
		kbm.knowledge_type = KB_KTYPE_PREDICATE;
		kbm.attribute_name = pname;
		/*
		for ( auto it=params.begin( ) ; it!=params.end( ) ; ++it )
			request.knowledge.values.push_back( diagnostic_msgs::KeyValue( it->first, it->second ) );
		*/
		
		for ( auto it=params.begin( ) ; it!=params.end( ) ; ++it )
		{
			diagnostic_msgs::KeyValue kv;
			kv.key = it->first;
			kv.value = it->second;
			kbm.values.push_back( kv );
		}
		
		query.request.knowledge.push_back( kbm );
		
		// call the query service
		if( !cl_query->call( query ) ) 
		{ 
			TERR( "unable to make a service request -- failed calling service " 
				<< LOGSQUARE( SERVICE_QUERY ) 
				<< (!cl_query->exists( ) ? " -- it seems not opened" : "") );
			return false;
		}
		
		// always true!
		// return ( query.response.results.size( ) > 0 );
		
		return query.response.all_true;
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
	
	TLOG( "Opening client " << LOGSQUARE( SERVICE_KB_UPDATE ) << "..." );
	ros::ServiceClient tcl_kb_update = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>( SERVICE_KB_UPDATE );
	if( !tcl_kb_update.waitForExistence( ros::Duration( TIMEOUT_KB_UPDATE ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_KB_UPDATE << "s) " );
		return 0;
	}
	cl_kb_update = &tcl_kb_update;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_KB_UPDATE ) << "... OK" );
	
	TLOG( "Opening client " << LOGSQUARE( SERVICE_KB_GET_FLUENT ) << "..." );
	ros::ServiceClient tcl_kb_get_fluent = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>( SERVICE_KB_GET_FLUENT );
	if( !tcl_kb_get_fluent.waitForExistence( ros::Duration( TIMEOUT_KB_GET_FLUENT ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_KB_GET_FLUENT << "s) " );
		return 0;
	}
	cl_kb_get_fluent = &tcl_kb_get_fluent;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_KB_GET_FLUENT ) << "... OK" );
	
	TLOG( "Opening client " << LOGSQUARE( SERVICE_QUERY ) << "..." );
	ros::ServiceClient tcl_query = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>( SERVICE_QUERY );
	if( !tcl_query.waitForExistence( ros::Duration( TIMEOUT_QUERY ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_QUERY << "s) " );
		return 0;
	}
	cl_query = &tcl_query;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_QUERY ) << "... OK" );
	
	TLOG( "ready" );
	
	( test_load_and_run( ) ).spin( );
	
	return 0;
}


#ifndef __H_KB_TOOLS__
#define __H_KB_TOOLS__


#include "ros/ros.h"

#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "diagnostic_msgs/KeyValue.h"

#include <string>

#ifndef __DEBUG_MACROS__
#define __DEBUG_MACROS__

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#endif


// KB query (predicates only)
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#define SERVICE_QUERY "/rosplan_knowledge_base/query_state"
#define TIMEOUT_QUERY 5

// KB update (fluents and predicates)
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#define SERVICE_KB_UPDATE "/rosplan_knowledge_base/update"
#define TIMEOUT_KB_UPDATE 5


// === KB macros

// kb action
#define KB_ADD_KNOWLEDGE 0
#define KB_DEL_KNOWLEDGE 2

// kb knowledge type
#define KB_KTYPE_FLUENT 2
#define KB_KTYPE_PREDICATE 1



/*
 * NOTES:
 * -	always check that a action succeeded with the method .ok()
 * -	supported only fluents with no arguments: the rosplan query doesn't work...
 * */
class kb_tools
{
public:
	
	// class constructor
	kb_tools( );
	
	// class destructor 
	~kb_tools( );
	
	// check if the last action succeeded or not
	bool ok( );
	
	
	
	
	// === GETTERS
	
	// get a predicate
	bool get_predicate( 
		const std::string& pname, 
		std::map<std::string, std::string>& params );
	
	// get a fluent (no params)
	float get_fluent( 
		const std::string fname );
	
	
	
	
	// === SETTERS
	
	// set a predicate
	bool set_predicate(
		const std::string& pname, 
		std::map<std::string, std::string>& params,
		bool pvalue );
	
	// set a fluent (no args)
	bool set_fluent(
		std::string fname,
		float fvalue );
	
protected:
	
	// build a query message (predicates only)
	rosplan_knowledge_msgs::KnowledgeQueryService request_query(
		const std::string& pname, 
		std::map<std::string, std::string>& params );
	
	// build a update message for predicates
	rosplan_knowledge_msgs::KnowledgeUpdateService request_update(
		const std::string pname, 
		std::map<std::string, std::string>& params, 
		bool value );

private:

	// node handle reference
	ros::NodeHandle nh;
	
	// last action success or not
	bool success;
	
	// predicates query client handle
	ros::ServiceClient *cl_query;
	
	// update clent handle
	ros::ServiceClient *cl_kb_update;
	
	// open the services with the knowledge base
	void open_services( );
};





#endif

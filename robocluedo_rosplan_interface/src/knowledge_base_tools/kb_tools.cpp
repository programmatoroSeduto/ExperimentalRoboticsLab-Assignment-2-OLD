
/********************************************//**
*  
* @file kb_tools
* @brief implementation of the class \ref kb_tools
* 
* @authors Fracesco Ganci
* @version v1.0
*  
* @see kb_tools.h
* 
***********************************************/

#include "knowledge_base_tools/kb_tools.h"



// class constructor
kb_tools::kb_tools( ) : 
	success( true )
{
	// open the services
	this->open_services( );
}


// class destructor 
~kb_tools( ) 
{
	// empty
};


// check if the last action succeeded or not
bool kb_tools::ok( )
{
	return this-> success;
}




// === GETTERS

// get value of a predicate
bool kb_tools::get_predicate( 
	const std::string& pname, 
	std::map<std::string, std::string>& params )
{
	// query message
	rosplan_knowledge_msgs::KnowledgeQueryService query = this->request_query(
		pname, params );

	// call the query service
	if( !this->cl_query->call( query ) ) 
	{ 
		TERR( "unable to make a service request -- failed calling service " 
			<< LOGSQUARE( SERVICE_QUERY ) 
			<< (!this->cl_query->exists( ) ? " -- it seems not opened" : "") );
		
		this->success = false;
		return false;
	}
	
	this->success = true;
	return query.response.all_true;
}


// get a fluent (no params)
float kb_tools::get_fluent( 
	const std::string fname )
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
		
		this->success = false;
		return 0.0;
	}
	
	this->success = true;
	return kbm.response.attributes[0].function_value;
}




// === SETTERS

// set a predicate
bool kb_tools::set_predicate(
	const std::string& pname, 
	std::map<std::string, std::string>& params,
	bool pvalue )
{
	// formulate the request
	rosplan_knowledge_msgs::KnowledgeUpdateService kbm = this->request_update(
		pname, params, pvalue );
		
	// send command
	if( !cl_kb_update->call( kbm ) ) 
	{ 
		TERR( "unable to make a service request -- failed calling service " 
			<< LOGSQUARE( SERVICE_KB_UPDATE ) 
			<< (!this->cl_kb_update->exists( ) ? " -- it seems not opened" : "") );
		
		this->success = false;
		return false;
	}
	
	this->success = true;
	return true;
}


// set a fluent (no args)
bool kb_tools::set_fluent(
	std::string fname,
	float fvalue )
{
	// TODO
}




// === PROTECTED METHODS

// build a query message (predicates only)
rosplan_knowledge_msgs::KnowledgeQueryService kb_tools::request_query(
	const std::string& pname, 
	std::map<std::string, std::string>& params )
{
	// query message
	rosplan_knowledge_msgs::KnowledgeQueryService query;
	rosplan_knowledge_msgs::KnowledgeItem kbm;

	kbm.knowledge_type = KB_KTYPE_PREDICATE;
	kbm.attribute_name = pname;

	for ( auto it=params.begin( ) ; it!=params.end( ) ; ++it )
	{
		diagnostic_msgs::KeyValue kv;
		kv.key = it->first;
		kv.value = it->second;
		kbm.values.push_back( kv );
	}

	query.request.knowledge.push_back( kbm );
	
	return query;
}


// build a update message for predicates only
rosplan_knowledge_msgs::KnowledgeUpdateService kb_tools::request_update(
	const std::string pname, 
	std::map<std::string, std::string>& params, 
	bool value )
{
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
	
	return kbm;
}




// === PRIVATE METHODS

// open the services with the knowledge base
void kb_tools::open_services( )
{
	// === Predicates Query service === //
	TLOG( "Opening client " << LOGSQUARE( SERVICE_QUERY ) << "..." );
	ros::ServiceClient tcl_query = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>( SERVICE_QUERY );
	if( !tcl_query.waitForExistence( ros::Duration( TIMEOUT_QUERY ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_QUERY << "s) " );
		return 0;
	}
	this->cl_query = &tcl_query;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_QUERY ) << "... OK" );
	
	
	// === update service === //
	TLOG( "Opening client " << LOGSQUARE( SERVICE_KB_UPDATE ) << "..." );
	ros::ServiceClient tcl_kb_update = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>( SERVICE_KB_UPDATE );
	if( !tcl_kb_update.waitForExistence( ros::Duration( TIMEOUT_KB_UPDATE ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_KB_UPDATE << "s) " );
		return 0;
	}
	this->cl_kb_update = &tcl_kb_update;
	TLOG( "Opening client " << LOGSQUARE( SERVICE_KB_UPDATE ) << "... OK" );
}

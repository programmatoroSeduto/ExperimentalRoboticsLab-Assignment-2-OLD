
/********************************************//**
*  
* @file kb_tools.cpp
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
kb_tools::kb_tools( bool pdebug_mode ) : 
	success( true )
{
	// set the verbosity level
	this->set_debug_mode( pdebug_mode );
	
	// open the services
	this->open_services( );
}


// class destructor 
kb_tools::~kb_tools( ) 
{
	// ...
};


// check if the last action succeeded or not
bool kb_tools::ok( )
{
	if( debug_mode ) TLOG( "kb_tools::ok( )" << ( this->success ? "" : " !!! FAILURE !!!" ) );
	
	return this->success;
}


// set the log verosity level
void kb_tools::set_debug_mode( bool db_mode )
{
	// set the mode
	this->debug_mode = db_mode;
	
	// notify it
	if( db_mode ) TLOG( "kb_tools::set_debug_mode : DEBUG MODE ENABLED" );
	else TLOG( "kb_tools::set_debug_mode : debug mode not enabled" );
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
	
	if( debug_mode ) 
	{
		std::string pm = "";
		for ( auto it=params.begin( ) ; it!=params.end( ) ; ++it )
			pm += ", " + it->first + "=" + it->second;
		TLOG( "kb_tools::get_predicate" << "( " << pname << pm << " )"  );
	}

	// call the query service
	if( !this->cl_query.call( query ) ) 
	{ 
		TERR( "unable to make a service request -- failed calling service " 
			<< LOGSQUARE( SERVICE_QUERY ) 
			<< (!this->cl_query.exists( ) ? " -- it seems not opened" : "") );
		
		this->success = false;
		return false;
	}
	
	this->success = true;
	
	if( debug_mode ) TLOG( "kb_tools::get_predicate" << " CALL SUCCESS with return " 
		<< (query.response.all_true ? "true" : "false")  );
	return query.response.all_true;
}


// get a fluent (no params)
float kb_tools::get_fluent( 
	const std::string fname,
	std::map<std::string, std::string>& params )
{
	// prepare command
	rosplan_knowledge_msgs::GetAttributeService kbm;
	kbm.request.predicate_name = fname;
	
	if( debug_mode ) TLOG( "kb_tools::get_fluent(" << fname << ")"  );

	// call the service
	if( !cl_kb_get_fluent.call( kbm ) ) 
	{ 
		TERR( "unable to make a service request -- failed calling service " 
			<< LOGSQUARE( SERVICE_KB_GET_FLUENT ) 
			<< (!cl_kb_get_fluent.exists( ) ? " -- it seems not opened" : "") );
		
		this->success = false;
		return 0.0;
	}
	
	/*
	this->success = true;
	
	if( debug_mode ) TLOG( "kb_tools::get_fluent(" << " CALL SUCCESS with return " 
		<< kbm.response.attributes[0].function_value 
		<< " (size=" << kbm.response.attributes.size() << ")"  );
	return kbm.response.attributes[0].function_value;
	*/
	
	// find the value of the fluent
	float res = this->read_fluents_list( fname, params, kbm.response );
	
	if( debug_mode ) TLOG( "kb_tools::get_fluent(" << " CALL SUCCESS with return " 
		<< res << " (success=" << (this->success ? "1" : "0") << ")" );
	return res;
}




// === SETTERS

// set a predicate
bool kb_tools::set_predicate(
	const std::string& pname, 
	std::map<std::string, std::string>& params,
	bool pvalue )
{
	// formulate the request
	rosplan_knowledge_msgs::KnowledgeUpdateService kbm = 
		this->request_update( pname, params, pvalue );
	
	if( debug_mode ) 
	{
		std::string pm = "";
		for ( auto it=params.begin( ) ; it!=params.end( ) ; ++it )
			pm += ", " + it->first + "=" + it->second;
		TLOG( "kb_tools::set_predicate" << "( " << pname << pm << ", pvalue=" << (pvalue ? "true" : "false" ) << " )"  );
	}
		
	// send command
	if( !this->cl_kb_update.call( kbm ) ) 
	{ 
		TERR( "unable to make a service request -- failed calling service " 
			<< LOGSQUARE( SERVICE_KB_UPDATE ) 
			<< (!this->cl_kb_update.exists( ) ? " -- it seems not opened" : "") );
		
		this->success = false;
		return false;
	}
	
	this->success = kbm.response.success;
	if( debug_mode ) TLOG( "kb_tools::set_predicate" << " CALL SUCCESS with return " 
		<< (kbm.response.success ? "success" : "NOT success")  );
	return kbm.response.success;
}


// set a fluent (no args)
bool kb_tools::set_fluent(
	std::string fname,
	std::map<std::string, std::string>& params,
	float fvalue )
{
	// prepare command
	rosplan_knowledge_msgs::KnowledgeUpdateService kbm = 
		this->request_update( fname, params, fvalue );
	
	if( debug_mode ) 
	{
		std::string pm = "";
		for ( auto it=params.begin( ) ; it!=params.end( ) ; ++it )
			pm += ", " + it->first + "=" + it->second;
		TLOG( "kb_tools::set_fluent(" << fname << pm << ", fvalue=" << fvalue << ")"  );
	}

	// send the command
	if( !this->cl_kb_update.call( kbm ) ) 
	{ 
		TERR( "unable to make a service request -- failed calling service " 
			<< LOGSQUARE( SERVICE_KB_UPDATE ) 
			<< (!cl_kb_update.exists( ) ? " -- it seems not opened" : "") );
		
		this->success = false;
		return false;
	}
	
	this->success = kbm.response.success;
	if( debug_mode ) TLOG( "kb_tools::set_predicate" << " CALL SUCCESS with return " 
		<< (kbm.response.success ? "success" : "NOT success")  );
	return kbm.response.success;
}



// === OTHER QUERIES

int kb_tools::exists_predicate(
		const std::string& pname, 
		std::map<std::string, std::string>& params )
{
	// query message
	rosplan_knowledge_msgs::GetAttributeService query;
	query.request.predicate_name = pname;
	
	if( debug_mode ) 
	{
		std::string pm = "";
		for ( auto it=params.begin( ) ; it!=params.end( ) ; ++it )
			pm += ", " + it->first + "=" + it->second;
		TLOG( "kb_tools::exists_predicate" << "( " << pname << pm << " )"  );
	}

	// call the query service
	if( !this->cl_query_2.call( query ) ) 
	{ 
		TERR( "unable to make a service request -- failed calling service " 
			<< LOGSQUARE( SERVICE_QUERY ) 
			<< (!this->cl_query_2.exists( ) ? " -- it seems not opened" : "") );
		
		this->success = false;
		return false;
	}
	
	// count the results
	int res = 0;
	// for( auto it=query.response.attributes.begin( ) ; it!=query.response.attributes.end( ) ; ++it )
	
	if( params.size( ) == 0 ) res = query.response.attributes.size( );
	else
	{
		bool found = false;
		int n_max_params = query.response.attributes.size( );
		
		for( auto it=query.response.attributes.begin( ) ; it!=query.response.attributes.end( ) ; ++it )
		{
			// check parameters (brute force)
			int n_params = n_max_params;
			for( auto it_par=params.begin() ; it_par!=params.end() ; ++it_par ) // for each parameter...
			{
				int newval = n_params - 1;
				for( auto it_k=it->values.begin( ) ; it_k!=it->values.end( ) ; ++it_k ) // ... and for each value in the knowledge item ...
				{
					if( ( it_par->first ==  it_k->key ) && ( it_par->second ==  it_k->value ) )
					{
						--n_params;
						break;
					}
				}
				
				if( newval < n_params )
				{
					break;
				}
				else if( n_params == 0 )
				{
					found = true;
					break;
				}
			}
			
			if( found )
			{
				++res;
				found = false;
			}
		}
	}
	
	if( debug_mode ) 
		TLOG( "kb_tools::get_predicate" << " CALL SUCCESS with return " << res );
	
	return res;
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


// build a update message for fluents only
rosplan_knowledge_msgs::KnowledgeUpdateService kb_tools::request_update(
	const std::string fname, 
	std::map<std::string, std::string>& params,
	float fvalue )
{
	rosplan_knowledge_msgs::KnowledgeUpdateService kbm;

	kbm.request.update_type = KB_ADD_KNOWLEDGE;
	kbm.request.knowledge.knowledge_type = KB_KTYPE_FLUENT;
	kbm.request.knowledge.attribute_name = fname;
	kbm.request.knowledge.function_value = fvalue;
	
	for ( auto it=params.begin( ) ; it!=params.end( ) ; ++it )
	{
		diagnostic_msgs::KeyValue kv;
		kv.key = it->first;
		kv.value = it->second;
		kbm.request.knowledge.values.push_back( kv );
	}
	
	return kbm;
}


// look for a particular fluent in a list of fluents
float kb_tools::read_fluents_list(
	const std::string fname, 
	std::map<std::string, std::string>& params,
	rosplan_knowledge_msgs::GetAttributeService::Response& res )
{
	bool found = false;
	float fvalue = 0.0;
	int n_max_params = params.size( );
	
	// simple case: zero-args functional
	if( n_max_params == 0 )
	{
		// the response contains just one knowledge item
		if( res.attributes.size( ) > 0 )
		{
			if( ( res.attributes[0].attribute_name == fname ) && ( res.attributes[0].knowledge_type == KB_KTYPE_FLUENT ) )
			{
				this->success = true;
				return res.attributes[0].function_value;
			}
			else
			{
				this->success = false;
				return fvalue;
			}
		}
		else
		{
			this->success = false;
			return fvalue;
		}
	}
	
	for( auto it_kni=res.attributes.begin() ; it_kni!=res.attributes.end() ; ++it_kni )
	{
		//check for name and type
		if( it_kni->knowledge_type != KB_KTYPE_FLUENT ) 
			continue;
		else if( it_kni->attribute_name != fname ) 
			continue;
		
		// check parameters (brute force)
		int n_params = n_max_params;
		for( auto it_par=params.begin() ; it_par!=params.end() ; ++it_par ) // for each parameter...
		{
			int newval = n_params - 1;
			for( auto it_k=it_kni->values.begin() ; it_k!=it_kni->values.end() ; ++it_k ) // ... and for each value in the knowledge item ...
			{
				if( ( it_par->first ==  it_k->key ) && ( it_par->second ==  it_k->value ) )
				{
					--n_params;
					break;
				}
			}
			
			if( newval < n_params )
			{
				break;
			}
			else if( n_params == 0 )
			{
				found = true;
				break;
			}
		}
		
		if( found )
		{
			fvalue = it_kni->function_value;
			break;
		}
	}
	
	this->success = found;
	return fvalue;
}




// === PRIVATE METHODS

// open the services with the knowledge base
void kb_tools::open_services( )
{
	// === Predicates Query service === //
	TLOG( "Opening client " << LOGSQUARE( SERVICE_QUERY ) << "..." );
	this->cl_query = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>( SERVICE_QUERY );
	if( !this->cl_query.waitForExistence( ros::Duration( TIMEOUT_QUERY ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_QUERY << "s) " );
		
		this->success = false;
		return;
	}
	TLOG( "Opening client " << LOGSQUARE( SERVICE_QUERY ) << "... OK" );
	
	
	// === another Predicate query service === //
	TLOG( "Opening client " << LOGSQUARE( SERVICE_QUERY_2 ) << "..." );
	this->cl_query_2 = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>( SERVICE_QUERY_2 );
	if( !this->cl_query_2.waitForExistence( ros::Duration( TIMEOUT_QUERY_2 ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_QUERY_2 << "s) " );
		
		this->success = false;
		return;
	}
	TLOG( "Opening client " << LOGSQUARE( SERVICE_QUERY_2 ) << "... OK" );
	
	
	// === Fluents Query service === //
	TLOG( "Opening client " << LOGSQUARE( SERVICE_KB_GET_FLUENT ) << "..." );
	this->cl_kb_get_fluent = nh.serviceClient<rosplan_knowledge_msgs::GetAttributeService>( SERVICE_KB_GET_FLUENT );
	if( !this->cl_kb_get_fluent.waitForExistence( ros::Duration( TIMEOUT_KB_GET_FLUENT ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_KB_GET_FLUENT << "s) " );
		
		this->success = false;
		return;
	}
	TLOG( "Opening client " << LOGSQUARE( SERVICE_KB_GET_FLUENT ) << "... OK" );
	
	
	// === update service === //
	TLOG( "Opening client " << LOGSQUARE( SERVICE_KB_UPDATE ) << "..." );
	this->cl_kb_update = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateService>( SERVICE_KB_UPDATE );
	if( !this->cl_kb_update.waitForExistence( ros::Duration( TIMEOUT_KB_UPDATE ) ) )
	{
		TERR( "unable to contact the server - timeout expired (" << TIMEOUT_KB_UPDATE << "s) " );
		
		this->success = false;
		return;
	}
	TLOG( "Opening client " << LOGSQUARE( SERVICE_KB_UPDATE ) << "... OK" );
}

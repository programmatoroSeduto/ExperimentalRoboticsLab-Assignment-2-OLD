
/********************************************//**
*  
* @file robocluedo_kb_tools.cpp
* @brief implementation of the class \ref robocluedo_kb_tools
* 
* @authors Fracesco Ganci
* @version v1.0
*  
* @see kb_tools.h
* @see robocluedo_kb_tools.h
* 
***********************************************/

#include "knowledge_base_tools/robocluedo_kb_tools.h"



// class constructor
robocluedo_kb_tools::robocluedo_kb_tools( bool debug_mode ) : 
	kb_tools( debug_mode )
{
	std::map<std::string, std::string> params;
	
	// number of IDs in the problem
	this->max_num_of_hints_from_problem = this->get_fluent( "number-of-ids-in-the-problem", params );
	if( !this->ok( ) )
	{
		// communication error
		return;
	}
}


// class destructor
robocluedo_kb_tools::~robocluedo_kb_tools( )
{
	// ...
}

// get the maximum number of IDs
int robocluedo_kb_tools::get_num_of_ids( )
{
	return this->max_num_of_hints_from_problem;
}

// get the number of active IDs
int robocluedo_kb_tools::get_open_ids( )
{
	std::map<std::string, std::string> params;
	int res = this->get_fluent( "h-count-open", params );
	
	if( this->ok( ) )
		return res;
	else
		return -1;
}

// get the number of complete IDs
int robocluedo_kb_tools::get_complete_ids( )
{
	std::map<std::string, std::string> params;
	int res = this->get_fluent( "h-count-complete", params );
	
	if( this->ok( ) )
		return res;
	else
		return -1;
}

// get the number of discarded IDs
int robocluedo_kb_tools::get_discard_ids( )
{
	std::map<std::string, std::string> params;
	int res = this->get_fluent( "h-count-discard", params );
	
	if( this->ok( ) )
		return res;
	else
		return -1;
}




// === HYPOTHESES MANAGEMENT === //

// check the current classification of one hypothesis
hypothesis_class robocluedo_kb_tools::get_status_of_hypothesis( int id )
{
	// query
	std::string hname = "id" + std::to_string( id );
	std::map<std::string, std::string> params;
	params["id"] = hname;
	
	bool res_ok = true;
	
	if( debug_mode )
		TLOG( "robocluedo_kb_tools::get_status_of_hypothesis" << "( " << id << " )"  );
	
	// classification
	bool is_discard = this->get_predicate( "h-discard", params );
	if( !this->ok( ) )
	{
		if( debug_mode )
			TWARN( "robocluedo_kb_tools::get_status_of_hypothesis " << "(h-discard) returning UNKNOWN"  );
		return hypothesis_class::UNKNOWN;
	}
	bool is_open = this->get_predicate( "h-open", params );
	if( !this->ok( ) )
	{
		if( debug_mode )
			TWARN( "robocluedo_kb_tools::get_status_of_hypothesis " << "(h-open) returning UNKNOWN"  );
		return hypothesis_class::UNKNOWN;
	}
	bool is_complete = this->get_predicate( "h-complete", params );
	if( !this->ok( ) )
	{
		if( debug_mode )
			TWARN( "robocluedo_kb_tools::get_status_of_hypothesis " << "(h-complete) returning UNKNOWN"  );
		return hypothesis_class::UNKNOWN;
	}
	
	// consistency check for redundancy
	if( (is_open && is_discard) ||
		(is_open && is_complete) ||
		(is_discard && is_open && is_complete) )
	{
		if( debug_mode )
			TWARN( "robocluedo_kb_tools::get_status_of_hypothesis " << "consistency check FAILED returning UNCONSISTENT_REDUNDANT"  );
		return hypothesis_class::UNCONSISTENT_REDUNDANT;
	}
		
	// final classification
	if( is_discard )
	{
		if( debug_mode )
			TLOG( "robocluedo_kb_tools::get_status_of_hypothesis " << "SUCCESS with return " << "DISCARD"  );
		return hypothesis_class::DISCARD;
	}
	else if( is_open )
	{
		if( debug_mode )
			TLOG( "robocluedo_kb_tools::get_status_of_hypothesis " << "SUCCESS with return " << "OPEN"  );
		return hypothesis_class::OPEN;
	}
	else if( is_complete )
	{
		if( debug_mode )
			TLOG( "robocluedo_kb_tools::get_status_of_hypothesis " << "SUCCESS with return " << "COMPLETE"  );
		return hypothesis_class::COMPLETE;
	}
	else
	{
		if( debug_mode )
			TWARN( "robocluedo_kb_tools::get_status_of_hypothesis " << "FAILED with return " << "UNCONSISTENT_NO_CLASS"  );
		return hypothesis_class::UNCONSISTENT_NO_CLASS;
	}
}


// update the classification of one hypothesis
bool robocluedo_kb_tools::update_hypothesis( int id, hypothesis_class& new_type )
{
	// query
	std::string hname = "id" + std::to_string( id );
	std::map<std::string, std::string> params;
	params["id"] = hname;
	
	if( debug_mode )
		TLOG( "robocluedo_kb_tools::update_hypothesis" << "( " << id << " )"  );
	
	// check the current status of the hypothesis
	hypothesis_class prev_status = this->get_status_of_hypothesis( id );
	new_type = prev_status;
	int who = this->get_fluent( "h-count-who", params );
	if( !this->ok( ) ) 
	{
		if( debug_mode )
			TLOG( "robocluedo_kb_tools::update_hypothesis " << "(h-count-who ?id=" << hname << ") FAILED" );
		return false;
	}
	int where = this->get_fluent( "h-count-where", params );
	if( !this->ok( ) ) 
	{
		if( debug_mode )
			TLOG( "robocluedo_kb_tools::update_hypothesis " << "(h-count-where ?id=" << hname << ") FAILED" );
		return false;
	}
	int what = this->get_fluent( "h-count-what", params );
	if( !this->ok( ) ) 
	{
		if( debug_mode )
			TLOG( "robocluedo_kb_tools::update_hypothesis " << "(h-count-what ?id=" << hname << ") FAILED" );
		return false;
	}
	
	// get the current counters (to update)
	int num_open = this->get_open_ids( );
	if( num_open < 0 ) 
	{
		if( debug_mode )
			TLOG( "robocluedo_kb_tools::update_hypothesis " << "(get_open_ids) FAILED" );
		return false;
	}
	int num_complete = this->get_complete_ids( );
	if( num_complete < 0 ) 
	{
		if( debug_mode )
			TLOG( "robocluedo_kb_tools::update_hypothesis " << "(get_complete_ids) FAILED" );
		return false;
	}
	int num_discard = this->get_discard_ids( );
	if( num_discard < 0 ) 
	{
		if( debug_mode )
			TLOG( "robocluedo_kb_tools::update_hypothesis " << "(get_discard_ids) FAILED" );
		return false;
	}
	
	// hypothesis update
	switch( prev_status )
	{	
	case hypothesis_class::UNCONSISTENT_NO_CLASS :
	case hypothesis_class::UNCONSISTENT_REDUNDANT :
		// there's something wrong with the ontology!
		if( debug_mode )
			TWARN( "robocluedo_kb_tools::update_hypothesis " << "trying to update a unconsistent hypothesis" );
		return false;
	break;
	
	case hypothesis_class::DISCARD :
		// nothig to do: already discarded
		if( debug_mode )
			TLOG( "robocluedo_kb_tools::update_hypothesis " << "hypothesis already discarded, returning" );
		return true;
	break;
	
	case hypothesis_class::COMPLETE :
	{
		if( (who > 1) || (where > 1) || (what > 1) ) // check for inconsistency
		{
			if( debug_mode )
				TWARN( "robocluedo_kb_tools::update_hypothesis " << "trying to update a unconsistent hypothesis" );
			
			this->set_hypothesis_class( id, 
				hypothesis_class::DISCARD );
			
			--num_complete;
			++num_discard;
			
			if( debug_mode )
				TLOG( "robocluedo_kb_tools::update_hypothesis " << "COMPLETE->DISCARD num_complete=" << num_complete << " num_discard=" << num_discard );
			
			return this->set_counters_status( num_open, num_complete, num_discard );
		}
		else
		{
			// nothing to do: the hypothesis is still complete
			if( debug_mode )
				TLOG( "robocluedo_kb_tools::update_hypothesis " << "(COMPLETE) nothing to do: the hypothesis is still complete, returning" );
			return true; 
		}
	}
	break;
	
	case hypothesis_class::OPEN :
	{
		if( (who > 1) || (where > 1) || (what > 1) ) // check for inconsistency
		{
			this->set_hypothesis_class( id, hypothesis_class::DISCARD );
			
			--num_open;
			++num_discard;
			
			if( debug_mode )
				TLOG( "robocluedo_kb_tools::update_hypothesis " << "OPEN->DISCARD num_open=" << num_open << " num_discard=" << num_discard );
				
			return this->set_counters_status( num_open, num_complete, num_discard );
		}
		else if( (who == 1) && (where == 1) && (what == 1) ) // check for complete hypothesis
		{
			this->set_hypothesis_class( id, hypothesis_class::COMPLETE );
			
			--num_open;
			++num_complete;
			
			if( debug_mode )
				TLOG( "robocluedo_kb_tools::update_hypothesis " << "OPEN->COMPLETE num_open=" << num_open << " num_complete=" << num_complete );
			
			return this->set_counters_status( num_open, num_complete, num_discard );
		}
		else
		{
			// nothing to do
			if( debug_mode )
				TLOG( "robocluedo_kb_tools::update_hypothesis " << "(OPEN) nothing to do, returning" );
			return true;
		}
	}
	break;
	
	default:
		// just fom completeness: it can't happen
		return false;
	}
	
}




// === PRIVATE METHODS === //

// set a particular class for that hypothesis
bool robocluedo_kb_tools::set_hypothesis_class( 
	int id, hypothesis_class new_class )
{
	bool res = true;
	
	std::string hname = "id" + std::to_string( id );
	std::map<std::string, std::string> params;
	params["id"] = hname;
	
	// first step : update the predicates status
	switch( new_class )
	{
	case OPEN:
		res = this->set_predicate( "h-open", params, true )
			&& this->set_predicate( "h-complete", params, false )
			&& this->set_predicate( "h-discard", params, false ) ;
	break;
	case COMPLETE:
		res = this->set_predicate( "h-open", params, false )
			&& this->set_predicate( "h-complete", params, true )
			&& this->set_predicate( "h-discard", params, false ) ;
	break;
	case DISCARD:
		res = this->set_predicate( "h-open", params, false )
			&& this->set_predicate( "h-complete", params, false )
			&& this->set_predicate( "h-discard", params, true ) ;
	break;
	default:
		// update request not acceptable
		return false;
	}
	
	return res;
}


// update the counters in the problem
bool robocluedo_kb_tools::set_counters_status( 
	int n_open, int n_complete, int n_discard )
{
	std::map<std::string, std::string> params;
	
	// check the consistency of the update before starting!
	if( (n_open + n_complete + n_discard) != this->get_num_of_ids( ) )
		// something went wrong during the update!
		return false;
	
	// set the status of the fluents
	if( !this->set_fluent( "h-count-open", params, n_open ) )
	{
		return false;
	}
	if( !this->set_fluent( "h-count-complete", params, n_complete ) )
	{
		return false;
	}
	if( !this->set_fluent( "h-count-discard", params, n_discard ) )
	{
		return false;
	}
	
	return true;
}

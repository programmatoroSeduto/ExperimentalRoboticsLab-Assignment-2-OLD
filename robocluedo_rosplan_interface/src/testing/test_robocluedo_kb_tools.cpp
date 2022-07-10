
/********************************************//**
*  
* @file test_robocluedo_kb_tools.cpp
* 
* @brief a simple module for testing the robocluedo_kb_tools interface
* 
* @authors francesco ganci
* @version v1.0
* 
* ... more details
*  
* @see header the Header
* 
***********************************************/

#include "ros/ros.h"
#include "knowledge_base_tools/robocluedo_kb_tools.h"

#define NODE_NAME "test_robocluedo_kb_tools"

#ifndef __DEBUG_MACROS__
	#define __DEBUG_MACROS__

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#endif

#include <string>
#include <map>
#include <signal.h>

class test_robocluedo_kb_tools : public robocluedo_kb_tools
{
public:
	
	// node constructor
	test_robocluedo_kb_tools( ) :
		robocluedo_kb_tools( true );
	{
		// ...
	}
	
	// main functionality of the class
	void spin( )
	{
		/*
		 * il test parte dallo stato iniziale normalissimo: nessuna ipotesi 
		 * smarcata, tutte attive, col robot in mezzo alla stanza e sistema
		 * non inizializzato. 
		 * */
		
		// === cardinalità degli insiemi di ipotesi
		TLOG( "cardinalities of hypotheses classes ... " );
		int num_ids = this->get_num_of_ids( );
		if( num_ids < 0 )
		{
			TWARN( "num_ids=" << num_ids );
			return;
		}
		int num_open = this->get_open_ids( );
		if( num_open < 0 )
		{
			TWARN( "num_open=" << num_open );
			return;
		}
		int num_complete = this->get_complete_ids( );
		if( num_complete < 0 )
		{
			TWARN( "num_complete=" << num_complete );
			return;
		}
		int num_discard = this->get_discard_ids( );
		if( num_discard < 0 )
		{
			TWARN( "num_discard=" << num_discard );
			return;
		}
		this->print_counting( num_ids, num_open, num_complete, num_discard );
		
		
		
		TLOG( "checking solution existence using counters ... " );
		this->test_solvable( num_ids, num_open, num_complete, num_discard );
		
		
		TLOG( "checking hypotheses status ... " );
		this->print_hyp_status( num_ids );
		
		
		/*
		 * da qui iniziamo a simulare la ricezione di hint da parte del sistema.
		 * ci si aspetta che l'interfaccia reagisca cambiando opportunamente lo
		 * stato delle ipotesi alterate di volta in volta. 
		 * */
		
		TLOG( "making ID1 complete ... " );
		this->make_ID_complete( 1 );
		TLOG( "checking --- ID1 complete --- " );
		this->print_hyp_status( num_ids );
	}
		
	
private:
	
	// ROS node handle
    ros::NodeHandle nh;
    
    // conta gli ID e stampa a video
    void print_counting( int num_ids, int num_open, int num_complete, int num_discard )
    {
		TLOG( "num_ids" << num_ids );
		TLOG( "num_open" << num_open );
		TLOG( "num_complete" << num_complete  );
		TLOG( "num_discard" << num_discard  );
	}
	
	// controlla se il problema è ancora risolvibile
	void test_solvable( int num_ids, int num_open, int num_complete, int num_discard )
	{
		if( num_discard >= num_ids )
			TLOG( "(num_discard=" << num_discard << ") >= (num_ids=" << num_ids << ") NOT SOLVABLE" );
		
		else if( ((num_complte + num_open) == 1) && (num_discard == (num_ids - 1)) )
			TLOG( "(num_complete + num_open)=" << (num_complete + num_open) << " && " <<
				"(num_discard=" << num_discard << ") == " << (num_ids - 1) << " SOLVABLE BY EXCLUSION" );
		
		else
			TLOG( "SOLVABLE" );
	}
	
	// stampa lo stato di tutte le ipotesi disponibili
	void print_hyp_status( int num_ids )
	{
		for( int id=1 ; id<=num_ids ; ++id )
		{
			// stato dell'ipotesi attuale
			hypothesis_class c = this->get_status_of_hypothesis( id );
			
			// stampa in base al valore di ritorno
			switch( c )
			{
			case hypothesis_class::UNCONSISTENT_NO_CLASS :
				TLOG( "id=" << id << " status=" << "UNCONSISTENT_NO_CLASS" );
			break;	
			case hypothesis_class::UNCONSISTENT_REDUNDANT :
				TLOG( "id=" << id << " status=" << "UNCONSISTENT_REDUNDANT" );
			break;
			case hypothesis_class::UNKNOWN :
				TLOG( "id=" << id << " status=" << "UNKNOWN" );
			break;
			case hypothesis_class::OPEN :
				TLOG( "id=" << id << " status=" << "OPEN" );
			break;
			case hypothesis_class::COMPLETE :
				TLOG( "id=" << id << " status=" << "COMPLETE" );
			break;
			case hypothesis_class::DISCARD :
				TLOG( "id=" << id << " status=" << "DISCARD" );
			break;
			}
		}
	}
	
	// fai diventare completo un ID
	void make_ID_complete( int id )
	{
		std::string hname = "ID" + id;
		std::map<std::string, std::string> params;
		params["id"] = hname;
		
		this->set_fluent( "h-count-who", params, 1 );
		this->set_fluent( "h-count-where", params, 1 );
		this->set_fluent( "h-count-what", params, 1 );
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
	
	TLOG( "ready" );
	( test_robocluedo_kb_tools( ) ).spin( );
	
	return 0;
}

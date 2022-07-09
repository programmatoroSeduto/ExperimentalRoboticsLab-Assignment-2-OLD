
/********************************************//**
 *  
 * \file file.ext
 * <div><b>ROS Node Name</b> 
 *      <ul><li>test_kb_tools</li></ul></div>
 * 
 * \brief testing module for the class \ref kb_tools
 * 
 * \authors Francesco Ganci
 * \version v1.0
 * 
 * <b>Description:</b> <br>
 * <p>
 * This node implements a simple sequence of tests on the interface
 * \ref kb_tools on a running knowledge base. 
 * </p>
 * 
 * <b>UML component</b><br>
 * (See ... the overal architecture, for further informations)<br>
 * <img src="" alt="TODO uml"/><br>
 * 
 * <b>Publishers:</b> <br>
 * <ul>
 *     <li>
 * 			<i>/topic</i> : file.msg <br>
 * 			... description 
 * 		</li>
 * </ul>
 * 
 * <b>Subscribers:</b> <br>
 * <ul>
 *     <li>
 * 			<i>/topic</i> : file.msg <br>
 * 			... description 
 * 		</li>
 * </ul>
 * 
 * <b>Services:</b> <br>
 * <ul>
 *     <li>
 * 			<i>/service</i> : file.srv <br>
 * 			... description 
 * 		</li>
 * </ul>
 * 
 * <b>Clients:</b> <br>
 * <ul>
 *     <li>
 * 			<i>/serv</i> : file.srv <br>
 * 			... reference to the implementation
 * 		</li>
 * </ul>
 * 
 * <b>Providing actions</b> <br>
 * <ul>
 *     <li>
 * 			<i>action_name</i> : file.action <br>
 * 			... description 
 * 		</li>
 * </ul>
 * 
 * <b>Using actions</b> <br>
 * <ul>
 *     <li>
 * 			<i>action_name</i> : file.action <br>
 * 			... description 
 * 		</li>
 * </ul>
 * 
 * <b>Hidden Services and Topics:</b> <br>
 * <ul>
 * 		<li>
 * 			( from ... : type ) <i>/channel</i> : type.format <br>
 * 			... reference to page
 * 		</li>
 * </ul>
 * 
 * <b>Parameters:</b> <br>
 * <ul>
 * 		<li>
 * 			[GET/SET] <i>/parameter</i> : type <br>
 * 			... description 
 * 		</li>
 * </ul>
 * 
 * <b>Test the code</b><br>
 * <code>
 * ...
 * </code>
 * 
 * <b>TODOs</b><br>
 * 
 ***********************************************/

#define NODE_NAME "test_kb_tools"
#ifndef __DEBUG_MACROS__
	#define __DEBUG_MACROS__ "__DEBUG_MACROS__"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#endif

#include "ros/ros.h"

#include "knowledge_base_tools/kb_tools.h"

#include <string>
#include <map>
#include <signal.h>
#include <time.h>


class test_kb_tools : public kb_tools
{
public:
	
	// node constructor
	test_kb_tools( ) :
		kb_tools( true )
	{
		srand( time(NULL) );
	}
	
	// main functionality of the class
	void spin( )
	{
		// params: only one parameter
		std::map<std::string, std::string> params_1;
		params_1["b"] = "b1";
		
		// params: empty
		std::map<std::string, std::string> params_2;
		
		// params: two args
		std::map<std::string, std::string> params_3;
		params_3["k1"] = "kk1";
		params_3["k2"] = "kk2";
		
		// === predicates: set and test == //
		// return value for predicates
		bool res = false;
		
		TLOG( "SET predicate (b-true b1)=1" );
		this->set_predicate( "b-true", params_1, true );
		if( !this->ok( ) ) return;
		
		TLOG( "GET predicate (b-true b1)" );
		res = this->get_predicate( "b-true", params_1 );
		if( !this->ok( ) ) return;
		else TLOG( "(expected '1') (b-true b1)=" << (res? "1" : "0") );
		
		TLOG( "SET (again) predicate (b-true b1)=1" );
		this->set_predicate( "b-true", params_1, true );
		if( !this->ok( ) ) return;
		
		TLOG( "GET predicate (b-true b1)" );
		res = this->get_predicate( "b-true", params_1 );
		if( !this->ok( ) ) return;
		else TLOG( "(expected '1') (b-true b1)=" << (res? "1" : "0") );
		
		TLOG( "SET predicate (b-true b1)=0" );
		this->set_predicate( "b-true", params_1, false );
		if( !this->ok( ) ) return;
		
		TLOG( "GET predicate (b-true b1)" );
		res = this->get_predicate( "b-true", params_1 );
		if( !this->ok( ) ) return;
		else TLOG( "(expected '0') (b-true b1)=" << (res? "1" : "0") );
		// === //
		
		
		// === fluents no args: set and test == //
		int randval = rand( ) / 10000.0;
		
		// return value for fluents
		float rval = 0.0;
		
		TLOG( "SET fluent (f-non-zero )=" << randval );
		this->set_fluent( "f-non-zero", params_2, randval );
		if( !this->ok( ) ) return;
		
		TLOG( "GET fluent (f-non-zero )" );
		rval = this->get_fluent( "f-non-zero", params_2 );
		if( !this->ok( ) ) return;
		else TLOG( "(expected '" << randval << "') (f-non-zero )=" << rval );
		// === //
		
		
		// === fluents 2 args: set and test == //
		randval = rand( ) / 10000.0;
		
		TLOG( "SET fluent (f-two-args kk1 kk2)=" << randval );
		this->set_fluent( "f-two-args", params_3, randval );
		if( !this->ok( ) ) return;
		
		TLOG( "GET fluent (f-two-args kk1 kk2)" );
		rval = this->get_fluent( "f-two-args", params_3 );
		if( !this->ok( ) ) return;
		else TLOG( "(expected '" << randval << "') (f-two-args kk1 kk2)=" << rval );
		// === //
		
	}
	
private:
	
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
	
	TLOG( "ready" );
	
	( test_kb_tools( ) ).spin( );
	
	return 0;
}

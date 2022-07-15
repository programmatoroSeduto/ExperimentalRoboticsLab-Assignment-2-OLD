
/********************************************//**
*  
* @file test_move_base.cpp
* @brief ...
* 
* @authors Francesco Ganci
* @version v1.0
* 
***********************************************/

#include "ros/ros.h"
#include "move_base_interface/move_base_interface.h"

#define NODE_NAME "test_move_base"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#include <string>
#include <map>
#include <signal.h>


class test_move_base : public move_base_interface
{
public:
	
	/// node constructor
	test_move_base( ) :
		move_base_interface( )
	{
		// ...
		
		/*
		for( int i=0; i<3; i++ )
			this->target_point[i] = 0.0;
		*/
	}
	
	/// main functionality of the class
	void spin( )
	{
		//    TEST 1
		// vai in una certa posizione e aspetta
		// base: -5.04918, 7.9953, 0.1
		/// @todo print?
		TLOG( "TO POINT -> (-1, -1)" );
		this->send_goal( true, -1, -1, 0 );
		
		//    TEST 2
		// vai in una certa posizione e controlla come varia lo stato
		/// @todo implementare test 2
		
		//    TEST 3
		// prova a raggiungere una certa posizione, poi cancella l'obiettivo
		/// @todo implementare test 3
	}
	
private:
	
	/// ROS node handle
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
	ros::AsyncSpinner spinner( 4 );
	spinner.start( );
	ros::NodeHandle nh;
	
	TLOG( "starting ... " );
	
	test_move_base test;
	
	TLOG( "ready" );
	
	test.spin( );
	
	return 0;
}

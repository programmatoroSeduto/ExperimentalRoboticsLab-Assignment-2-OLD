
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
		
		for( int i=0; i<3; i++ )
			this->target_point[i] = 0.0;
	}
	
	/// main functionality of the class
	void spin( )
	{
		//    TEST 1
		// vai in una certa posizione e aspetta
		// base: -5.04918, 7.9953, 0.1
		/// @todo print?
		this-send_goal( target_point[0], target_point[1], target_point[2], true );
		
		//    TEST 2
		// vai in una certa posizione e controlla come varia lo stato
		/// @todo implementare test 2
		
		//    TEST 3
		// prova a raggiungere una certa posizione, poi cancella l'obiettivo
		/// @todo implementare test 3
	}
	
	/********************************************//**
	 *  
	 * \brief posizione (x, y, z) da linea di comando del programma
	 * 
	 * @note la funzione si aspetta un vettore del tipo [name x y z],
	 * 	eventualmente anche con meno argomenti
	 * 
	 * @param argc il numero di elementi nel vettore
	 * @param argv (string[]) gli argomenti
	 * @param x (out)
	 * @param y (out)
	 * @param z (out)
	 * 
	 * @return true quando almeno un argomento viene parsato (x)
	 * 
	 * @note la funzione ritorna false quando incontra un errore
	 * 
	 * @note nessun ritorno quando il parsing fallisce
	 * 
	 * @note la funzione ritorna false anche quando non ci sono argomenti
	 * 	da parsare, array vuoto. 
	 * 
	 * @todo magari un metodo di implementazione più ... furbo?
	 * 
	 ***********************************************/
	bool parse_argv( int argc, char* argv[], int& x, int& y, int& z )
	{
		// valori temporanei (non alterare il riferimento prima di essere sicuro)
		int xt = x;
		int yt = y;
		int zt = z;
		
		// parsing
		if( argc <= 2 )
		{
			if( !this->is_number( argv[1] ) ) xt = atof( argv[1] );
			else return false;
			
			if( argc <= 3 )
			{
				if( this->is_number( argv[2] ) ) yt = atof( argv[2] );
				else return false;
				
				if( argc <= 4 )
				{
					if( this->is_number( argv[2] ) ) zt = atof( argv[3] );
				}
			}
		}
		else
			// nulla da parsare
			return false;
	
		// aggiornamento finale
		x = xt; y = yt; z = zt;
		return true;
	}
	
	/// aggiungi un target alla classe via command line
	bool argv_to_target( int argc, char* argv[] )
	{
		int x, y, z;
		bool res = this->parse_argv( argc, argv, x, y, z );
		
		if( res )
		{
			target_point[0] = x;
			target_point[1] = y;
			target_point[2] = z;
		}
		else
			return false;
	}
	
private:
	
	/// ROS node handle
    ros::NodeHandle nh;
    
    /// target da raggiungere
    float target_point[3];
	
	/// controlla se una stringas rappresenta un numero
	bool is_number( char* str )
	{
		// trasforma in stringa 
		std::string s( str );
		
		for (char const &c : s) 
		{
			if (std::isdigit(c) == 0) return false;
		}
		return true;
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
	ros::AsyncSpinner spinner( 4 );
	spinner.start( );
	ros::NodeHandle nh;
	
	TLOG( "starting ... " );
	
	test_move_base test;
	test.argv_to_target( argc, argv );
	
	TLOG( "ready" );
	
	test.spin( );
	
	return 0;
}


#include "ros/ros.h"
#include <signal.h>
#include <string>

#define NODE_NAME "manipulation_controller"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//include <moveit_visual_tools/moveit_visual_tools.h>
//include <moveit_msgs/DisplayRobotState.h>
//include <moveit_msgs/DisplayTrajectory.h>
//include <moveit_msgs/AttachedCollisionObject.h>
//include <moveit_msgs/CollisionObject.h>

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#define ARM_PLANNING_GROUP "arm_group"

// manipulation service 
#include "robocluedo_movement_controller_msgs/TipPosition.h"
#define SERVICE_MANIP "/tip_pos"
ros::ServiceServer *srv_manip;


class node_manipulation_controller
{
public:
	
	node_manipulation_controller(  ) : mgi( ARM_PLANNING_GROUP )
	{
		// init MoveIt groups
		// static const std::string PLANNING_GROUP = ARM_PLANNING_GROUP;
		
		// MoveIt settings
		mgi.setPlanningTime(10.0);
		
		(ros::Duration(1.0)).sleep( );
		
		// set the initial position of the arm
		mgi.setNamedTarget( "init" );
		mgi.move( );
	}
	
	void spin( int n_loops_per_sec = 1 )
	{
		ros::waitForShutdown( );
	}
	
	bool cbk_manip( 
		robocluedo_movement_controller_msgs::TipPosition::Request& req, 
		robocluedo_movement_controller_msgs::TipPosition::Response& res )
	{
		if( req.set_home )
		{
			// move to home position
			mgi.setNamedTarget( "init" );
		}
		else
		{
			// move the tip to a certain point
			mgi.setNamedTarget( "collect_hint" );
		}
		
		mgi.move( );
		
		res.success = true;
		return true;
	}
	
private:
	
	// ROS node handle
    ros::NodeHandle nh;
	
	// moveIt group name
	std::string planning_group;
	
	// planning interface
	moveit::planning_interface::MoveGroupInterface mgi;
	
	// previously generated plan
	moveit::planning_interface::MoveGroupInterface::Plan plan;
	
	// ...
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
	
	// required by moveIt: run the node as AsyncSpinner
	ros::AsyncSpinner spinner( 2 );
	spinner.start( );
	
	TLOG( "starting ... " );
	
	node_manipulation_controller node;
	
	TLOG( "Advertising service " << LOGSQUARE( SERVICE_MANIP ) << "..." );
	ros::ServiceServer tsrv_manip = nh.advertiseService( SERVICE_MANIP, &node_manipulation_controller::cbk_manip, &node );
	srv_manip = &tsrv_manip;
	TLOG( "Advertising service " << LOGSQUARE( SERVICE_MANIP ) << "... OK" );
	
	TLOG( "ready" );
	
	node.spin( );
	
	return 0;
}

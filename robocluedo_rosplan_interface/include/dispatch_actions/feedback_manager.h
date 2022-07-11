
/********************************************//**
*  
* @file feedback_manager.h
* @brief abstraction for the feedback publication
* 
* @authors Francesco Ganci
* @version v1.0
*  
* @see feedback_manager
* 
***********************************************/

#define NODE_NAME "feedback_manager"

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#include "ros/ros.h"

#include <string>
#include <map>
#include <vector>

#include "rosplan_dispatch_msgs/DispatchService.h"
/*
---
bool success
bool goal_achieved
*/

#define Q_SZ 1000
#define ACTION_NAME_UNKNOWN "UNKNOWN"

// action feedback topic (pub)
#include "robocluedo_rosplan_interface_msgs/ActionFeedback.h"
#define TOPIC_ACTION_FEEDBACK "/robocluedo/action_feedback"

class action_feedback_manager
{
public:
	
	// === BASE METHODS === //
	
	/********************************************//**
	 *  
	 * \brief constructor of action_feedback_manager
	 * 
	 * this function opens the publisher and performs some other
	 * intialization. 
	 * 
	 ***********************************************/
	action_feedback_manager( std::string act_name = ACTION_NAME_UNKNOWN );
	
	/// class destructor
	~action_feedback_manager( );
	
	
	
	// === FEEDBACK SYSTEM === //
	
	/********************************************//**
	 *  
	 * \brief knowledge base not in a consistent status
	 * 
	 * each field is false, except for 'kb_not_consistent' and (eventually)
	 * the field 'goal_achieved'. 
	 * 
	 * @param goal goal achieved or not?  
	 * 
	 * @returns the outcome of the publishing action
	 * 
	 * @note check fist 'problem_solvable' -> (not solvable) then check 
	 * 	'problem_solvable' -> (not solvable) and finally check
	 * 	'kb_not_consistent' (it is set true)
	 *	
	 * @note no need for replan
	 * 
	 ***********************************************/
	bool fb_unconsistent( std::vector<diagnostc_msgs::KeyValue> parameters, bool goal = false, std::string details = "" );
	
	/********************************************//**
	 *  
	 * \brief feedback, the problem is solvable
	 * 
	 * @param excl solvable by exclusion?
	 * 
	 * @returns the outcome of the publishing action
	 * 
	 * @note check fist 'problem_solvable' -> (solvable) then check 
	 * 	'by_exclusion' (true or false)
	 *	
	 * @note need for replan only if 'by_execution'
	 * 
	 ***********************************************/
	bool fb_solvable( std::vector<diagnostc_msgs::KeyValue> parameters, bool excl, std::string details = "" );
	
	/********************************************//**
	 *  
	 * \brief feedback, the problem is unsolvable
	 * 
	 * @returns the outcome of the publishing action
	 * 
	 * @note check fist 'problem_solvable' -> (not solvable) 
	 *	
	 * @note no need for replan: the mystery can't be solved, each option
	 * has been discarded during the investigation. Probably there's 
	 * something wrong with the oracle implementation if this case happens. 
	 * 
	 ***********************************************/
	bool fb_unsolvable( std::vector<diagnostc_msgs::KeyValue> parameters, std::string details = "" );
	
	/********************************************//**
	 *  
	 * \brief feedback, need for replan (other reasons)
	 * 
	 * @returns the outcome of the publishing action
	 * 
	 * @note check fist 'problem_solvable' -> (solvable) check 
	 * 	for 'need_replan' -> (true)
	 *	
	 * @note this is a very general signal: better to specify what it means. 
	 * 
	 ***********************************************/
	bool fb_replan( std::vector<diagnostc_msgs::KeyValue> parameters, std::string details = "" );
	
	/// hardware failure (if not navigation, then manipulation)
	bool fm_hw_failure( std::vector<diagnostc_msgs::KeyValue> parameters, 
		bool navigation, std::string details = "" );



private:
	
	/// ROS node handle
	ros::NodeHandle nh;
	
	/// the publisher handle
	ros::Publisher pub_action_feedback;
	
	/// the name of the action
	std::string action_name;
	
	/********************************************//**
	 *  
	 * \brief build a action feedback message
	 * 
	 * @param parameters directly from the propertied of the action
	 * 
	 * @returns the feedback message ready to be sent
	 * 
	 ***********************************************/
	robocluedo_rosplan_interface_msgs::ActionFeedback build_msg( 
		std::vector<diagnostc_msgs::KeyValue> parameters,
		bool problem_solvable = true,
		bool by_exclusion = false,
		bool kb_not_consistent = false,
		bool goal_achieved = false,
		bool need_replan = false,
		bool failure_nav_sys = false,
		bool failure_manipulation = false,
		std::string details = "" );
	
	/********************************************//**
	 *  
	 * \brief publish the feedback
	 * 
	 * @returns true if the publication succeeded, false otherwise
	 * 
	 ***********************************************/
	bool pub( robocluedo_rosplan_interface_msgs::ActionFeedback& msg );
}

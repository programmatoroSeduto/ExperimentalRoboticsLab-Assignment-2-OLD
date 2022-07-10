
/********************************************//**
 *  
 * \file robocluedo_kb_tools.h
 * <div><b>ROS Node Name</b> 
 *      <ul><li>robocluedo_kb_tools</li></ul></div>
 * \brief ...brief...
 * 
 * \authors ???
 * \version v1.0
 * 
 * <b>Description:</b> <br>
 * <p>
 * ...description
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

#ifndef __H_ROBOCLUEDO_KB_TOOLS__
#define __H_ROBOCLUEDO_KB_TOOLS__ "__H_ROBOCLUEDO_KB_TOOLS__"

#include "ros/ros.h"
#include "knowledge_base_tools/kb_tools.h"

#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "diagnostic_msgs/KeyValue.h"

#include <string>
#include <map>

#define NODE_NAME "robocluedo_kb_tools"

#ifndef __DEBUG_MACROS__
#define __DEBUG_MACROS__

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#endif


/********************************************//**
 *  
 * \class robocluedo_kb_tools
 * 
 * @brief more services for the robocluedo project!
 * 
 * this is a more specific implementation of the interface with the
 * knowledge base, tailored for the robocluedo project. <br>
 * 
 * this class, as \ref kb_tools does, aims at simplifying and optimizing
 * the operations upon the knowledge base. The main idea is the same applied
 * for the project <i>robocluedo_armor_interface</i>: given the storage 
 * system, the package implements a handful abstraction hiding the most
 * complex, contorted communication aspects. The interface also provides
 * a debug system, making simpler to detect errors and bugs of the 
 * implementation. 
 * 
 * @see kb_tools the base class
 * 
 * @note this class is implemented upon the PDDL code of robocluedo: the
 * class implementation knows the names of the predicates and the 
 * functions. If names would change, also the code should be reviewed. 
 * 
 * @todo avoid hardcoding of the predicates/fluents names using the macros
 * 
 ***********************************************/
class robocluedo_kb_tools : public kb_tools
{
public:

	/********************************************//**
	 * 
	 * \enum hypothesis_class
	 *  
	 * @brief classes of hypothesis
	 * 
	 * @note the value UNKNOWN is returned when the query fails. 
	 * 
	 * @note each hypothesis ID can belong to only one class
	 * at time. a ID can't have two classes at the same time.
	 * 
	 ***********************************************/
	static enum hypothesis_class
	{
		OPEN,			// open hypothesis
		COMPLETE,		// complete hypothesis
		DISCARD,		// discarded hypothesis
		UNKNOWN,		// service call failed
		UNCONSISTENT_NO_CLASS,	// problem inconsistent: no class for a hypID
		UNCONSISTENT_REDUNDANT	// problem not consistent: two classes or more for the same hypID
	};
	
	/********************************************//**
	 *  
	 * \brief class constructor
	 * 
	 * @param debug_mode verbose print or not?
	 * 
	 ***********************************************/
	robocluedo_kb_tools( bool debug_mode = KB_TOOLS_DEFAULT_DEBUG_MODE );
	
	/********************************************//**
	 *  
	 * \brief class destructor
	 * 
	 ***********************************************/
	~robocluedo_kb_tools( );
	
	/********************************************//**
	 *  
	 * @brief get the maximum number of IDs
	 * 
	 * @returns the maximum number of IDs in the problem, 
	 * <i>just the total amount of IDs</i>, not only the 
	 * active ones. 
	 * 
	 ***********************************************/
	int get_num_of_ids( );
	
	/********************************************//**
	 *  
	 * @brief get the number of open IDs
	 * 
	 * @returns the number of active IDs currently, or -1 in case of 
	 * failure in calling the service. 
	 * 
	 * @note remember to update the classification of the
	 * hypotheses before calling this function. 
	 * 
	 ***********************************************/
	int get_open_ids( );
	
	/********************************************//**
	 *  
	 * @brief get the number of complete IDs
	 * 
	 * @returns the number of complete IDs currently, or -1 in case of 
	 * failure in calling the service. 
	 * 
	 * @note remember to update the classification of the
	 * hypotheses before calling this function. 
	 * 
	 ***********************************************/
	int get_complete_ids( );
	
	/********************************************//**
	 *  
	 * @brief get the number of discarded IDs
	 * 
	 * @returns the number of discarded IDs currently, or -1 in case of 
	 * failure in calling the service. 
	 * 
	 * @note remember to update the classification of the
	 * hypotheses before calling this function. 
	 * 
	 ***********************************************/
	int get_discard_ids( );
	 
	 
	 
	 // === HYPOTHESES MANAGEMENT === //
	 
	 /********************************************//**
	 *  
	 * \brief check the current classification of one hypothesis
	 * 
	 * @param id (int) the ID of the hyothesis to inspect
	 * 
	 * @note this classification could be not correct: it should be 
	 * updated before the checking. 
	 * 
	 * @note this method also checks for the consistency of the problem
	 * formulation. In particular, the hypotheses have one constraint:
	 * each hypothesis must belong to only one class each time, and they 
	 * must always have a class.
	 * 
	 * @see hypothesis_class
	 * 
	 * @todo we're assuming here that the given hypothesis exists, but
	 * what about not valid hypotheses IDs? 
	 * 
	 * @todo negative IDs?
	 * 
	 ***********************************************/
	robocluedo_kb_tools::hypothesis_class get_status_of_hypothesis( int id );
	 
	 /********************************************//**
	 *  
	 * \brief update the classification of one hypothesis
	 * 
	 * This method performs first a request to the knowledge base: how many
	 * elements are linked to one field of that hypothesis? then, depending
	 * on the number of elements related to the fields, the hypothesis is
	 * classified. 
	 * 
	 * @param id (int) the ID of the hyothesis to inspect
	 * @param new_type (&hypothesis_class) the new type of the hypothesis 
	 * 	after the update
	 * 
	 * @returns true is the operation succeeded or not. 
	 * 
	 * @note a discarded hypothesis is not updated. 
	 * 
	 * @note we're assuming that, if a hint is correctly read, it is immediately
	 * counted; therefore this method doesn't need to count: it shall just take
	 * the fluents and evaluate the hypothesis according on those. 
	 * 
	 * @todo we're assuming here that the given hypothesis exists, but
	 * what about not valid hypotheses IDs? 
	 * 
	 * @todo it makes sense to have a simple vector storing the discarded 
	 * hypotheses as a cache, instead of asking to the knowledge base each
	 * time. 
	 * 
	 * @todo negative IDs?
	 * 
	 ***********************************************/
	bool update_hypothesis( int id, robocluedo_kb_tools::hypothesis_class& new_type );
	 
	/********************************************//**
	 *  
	 * \brief update the classification of one hypothesis
	 * 
	 * override of the original function which takes only one
	 * argument. 
	 * 
	 * @param id (int) the ID of the hyothesis to inspect
	 * 
	 * @returns true is the operation succeeded or not. 
	 * 
	 * @see update_hypothesis
	 * 
	 ***********************************************/
	override 
	bool update_hypothesis( int id );



private:
	
	/// the maximum number of hints
	int max_num_of_hints_from_problem;
	
	/********************************************//**
	 *  
	 * \brief set a particular class for that hypothesis
	 * 
	 * @param id (int) the ID of the hyothesis to inspect
	 * @param new_class (hypothesis_class) the new type of the hypothesis 
	 * 
	 * @returns true if the operation succeeded
	 * 
	 * @note the method can be used also to solve any inconsistency in
	 * the database concerning the hypotheses. 
	 * 
	 * @warning hyotheses classification and counting are two distinct
	 *  operations! the method update_hypothesis updates both, but 
	 * <i>don't use only this method without also updating the count!</i> 
	 * 
	 * @todo negative IDs?
	 * 
	 ***********************************************/
	bool set_hypothesis_class( 
		int id, 
		robocluedo_kb_tools::hypothesis_class new_class );
	
	/********************************************//**
	 *  
	 * \brief set a status for the counting fluents in the problem
	 * 
	 * @param n_open 
	 * 		the number of IDs classified as 'open'
	 * @param n_complete 
	 * 		the number of IDs classified as 'complete'
	 * @param n_discard
	 * 		the number of IDs classified as 'discard'
	 * 
	 * @returns (bool) if the operation succeeded or not
	 * 
	 * @note this method returns false if the sum of the three parameters
	 * is not equal to the total amount of the hypotheses in the problem. 
	 * it is a small protection mechanism. 
	 * 
	 ***********************************************/
	bool set_counters_status( int n_open, int n_complete, int n_discard );
};

#endif

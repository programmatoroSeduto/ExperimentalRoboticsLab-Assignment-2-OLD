
/********************************************//**
 *  
 * \file kb_tools.h
 * <div><b>ROS Node Name</b> 
 *      <ul><li>???ros_node???</li></ul></div>
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
 * @todo write documentation here!
 * 
 ***********************************************/

#ifndef __H_KB_TOOLS__
#define __H_KB_TOOLS__ "__H_KB_TOOLS__"

#include "ros/ros.h"

#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "diagnostic_msgs/KeyValue.h"

#include <string>
#include <map>

#define NODE_NAME "kb_tools"

#ifndef __DEBUG_MACROS__
#define __DEBUG_MACROS__

#define LOGSQUARE( str )  "[" << str << "] "
#define OUTLABEL          LOGSQUARE( NODE_NAME )
#define TLOG( msg )       ROS_INFO_STREAM( OUTLABEL << msg )
#define TWARN( msg )      ROS_WARN_STREAM( OUTLABEL << "WARNING: " << msg )
#define TERR( msg )       ROS_WARN_STREAM( OUTLABEL << "ERROR: " << msg )

#endif

/// default debug mode
#define KB_TOOLS_DEFAULT_DEBUG_MODE false


// KB query (predicates only)
#define SERVICE_QUERY "/rosplan_knowledge_base/query_state"
#define TIMEOUT_QUERY 60

// KB propositions query
#define SERVICE_QUERY_2 "/rosplan_knowledge_base/state/propositions"
#define TIMEOUT_QUERY_2 60

// KB update (fluents and predicates)
#define SERVICE_KB_UPDATE "/rosplan_knowledge_base/update"
#define TIMEOUT_KB_UPDATE 60

// KB query (fluents only)
#define SERVICE_KB_GET_FLUENT "/rosplan_knowledge_base/state/functions"
#define TIMEOUT_KB_GET_FLUENT 60


// === KB macros

// kb action
#define KB_ADD_KNOWLEDGE 0
#define KB_DEL_KNOWLEDGE 2

// kb knowledge type
#define KB_KTYPE_FLUENT 2
#define KB_KTYPE_PREDICATE 1



/*
 * NOTES:
 * -	always check that a action succeeded with the method .ok()
 * -	supported only fluents with no arguments: the rosplan query doesn't work...
 * */
/********************************************//**
 *  
 * \class kb_tools
 * 
 * \brief base interface with the ROS plan knowledge base
 * 
 * this interface can be considered a abstraction of the ROS plan knowledge
 * base. the knowledge base is seen as a database containing predicates 
 * and fluents, which can be obtaine by GET methods, and set using SET 
 * methods. 
 * 
 * Moreover, the class opens and manages all the services needed to 
 * perform such communication with the database, hence the node using this
 * kind of functonality doesn't need to manually open the required 
 * interfaces. 
 * 
 ***********************************************/
class kb_tools
{
public:
	
	/********************************************//**
	 *  
	 * \brief class constructor
	 * 
	 * @param pdebug_mode log/shell output verbosity level, @see set_debug_mode
	 * 
	 ***********************************************/
	kb_tools( bool pdebug_mode = KB_TOOLS_DEFAULT_DEBUG_MODE );
	
	/// class destructor 
	~kb_tools( );
	
	/********************************************//**
	 *  
	 * \brief check if the last action succeeded or not
	 * 
	 * \note this function should be called every time you perform a
	 * call to any service. 
	 * 
	 ***********************************************/
	bool ok( );
	
	/********************************************//**
	 *  
	 * \brief set the log verbosity level
	 * 
	 * with the debug mode disabled, the class will notify only
	 * the opening of the services; otherwise, each method contains a 
	 * lot of printable messages. 
	 * 
	 * @param dbmode if true, the class will print on the log/shell 
	 * <i>every operations</i> the system will perform. 
	 * 
	 ***********************************************/
	void set_debug_mode( bool dbmode );
	
	
	
	
	// === GETTERS
	
	/********************************************//**
	 *  
	 * \brief get value of a predicate
	 * 
	 * @param pname the name of the predicate to get
	 * 
	 * @param params the map of the parameters of the predicate
	 * to check 
	 * 
	 * @returns <ul>
	 * <li><b>true</b> if the predicate is true (and the call succeeded)</li>
	 * <li><b>false</b> if the predicate is false OR if the service call 
	 * failed, see \ref ok</li>
	 * 
	 * @note the boolean 'false' is returned also when the call fails, so
	 * remember to check the valdity of the value using \ref ok
	 * 
	 * @todo the method assumes that only one result is returned, which is
	 * true for fully defined queries, but what about partial queries?
	 * 
	 ***********************************************/
	bool get_predicate( 
		const std::string& pname, 
		std::map<std::string, std::string> params );
	
	/********************************************//**
	 *  
	 * \brief get value of a fluent
	 * 
	 * @param fname the name of the fluent to read
	 * 
	 * @returns (float) the float value of the fluent
	 * 
	 * @note the value '0.0' is returned also when the call fails.
	 * 
	 * @warning remember to check the success flag using \ref ok
	 * 
	 * @todo check that the list of returned fluents is not empty!
	 * 
	 ***********************************************/
	float get_fluent( 
		const std::string fname,
		std::map<std::string, std::string> params );
	
	
	
	
	// === SETTERS
	
	/********************************************//**
	 *  
	 * \brief set the truth value of a predicate
	 * 
	 * @param pname the name of the predicate to get
	 * 
	 * @param params the map of the parameters of the predicate
	 * to check 
	 * 
	 * @param pvalue the boolean value to set for that predicate
	 * 
	 * @returns (bool) <b>true</b> if the operation has gone well, <b>false</b>
	 * otherwise.
	 * 
	 * @note no need here to check the success of the call using \ref ok:
	 * the return value is sufficient to understand what's going on.
	 * 
	 ***********************************************/
	bool set_predicate(
		const std::string& pname, 
		std::map<std::string, std::string> params,
		bool pvalue );
	
	/********************************************//**
	 *  
	 * \brief set a fluent (no args)
	 * 
	 * @param fname the name of the predicate to get
	 * 
	 * @param fvalue the new value of the fluent
	 * 
	 * @returns (bool) <b>true</b> if the operation has gone well, <b>false</b>
	 * otherwise.
	 * 
	 * @note no need here to check the success of the call using \ref ok
	 * 
	 ***********************************************/
	bool set_fluent(
		std::string fname,
		std::map<std::string, std::string> params,
		float fvalue );
	
	
	
	
	// === OTHER QUERIES
	
	/********************************************//**
	 *  
	 * \brief check how many elements are in a partial predicate query
	 * 
	 * in this application, the query "how many predicates with this argument
	 * are true?" occurs often. this method can count how many elements are
	 * involved in a partial query. 
	 * 
	 * asking something like that means asking the system to find <i>how many 
	 * predicates with this arguments are true</i>. If the request is fully
	 * defined, there's at most one element; if the query contains a 
	 * number of arguments less than the complete list, the function
	 * will return how many predicates are true with these parameters. 
	 * 
	 * @param pname the name of the predicate to get
	 * @param params list of parameters, <b>even a partial list</b>
	 * 
	 * @returns (int) how many elements are true with these parameters for
	 * that particular predicate. 
	 * 
	 * @warning remember to check the success flag using \ref ok
	 * 
	 * @todo code review: avoid code duplication!
	 * 
	 ***********************************************/
	int exists_predicate(
		const std::string& pname, 
		std::map<std::string, std::string>& params );



protected:
	
	/// node handle reference
	ros::NodeHandle nh;

	/// debug mode status, @see set_debug_mode
	bool debug_mode; 
	
	/// last action success or not, @see ok
	bool success;
	
	/********************************************//**
	 *  
	 * \brief build a query message (predicates only)
	 * 
	 * the method returns a message ready for a simple predicate query.
	 * just give the name of the predicate you're searching, and its
	 * parameters. 
	 * 
	 * @param pname (string) the name of the predicate
	 * 
	 * @param params (std::map<std::string, std::string>&) the map of 
	 * parameters for the predicate
	 * 
	 * @returns a copy of the message <i>rosplan_knowledge_msgs::KnowledgeQueryService</i>,
	 * with the <i>.request</i> field ready for the service request. 
	 * 
	 ***********************************************/
	rosplan_knowledge_msgs::KnowledgeQueryService request_query(
		const std::string& pname, 
		std::map<std::string, std::string>& params );
	
	/********************************************//**
	 *  
	 * \brief build a update message for predicates
	 * 
	 * @param pname (string) the name of the predicate
	 * 
	 * @param params (std::map<std::string, std::string>&) the map of 
	 * parameters for the predicate
	 * 
	 * @param value (bool) the new value of the predicate
	 * 
	 * @returns a copy of the message <i>rosplan_knowledge_msgs::KnowledgeUpdateService</i>,
	 * with the <i>.request</i> field ready for the service request. 
	 * 
	 ***********************************************/
	rosplan_knowledge_msgs::KnowledgeUpdateService request_update(
		const std::string pname, 
		std::map<std::string, std::string>& params, 
		bool value );
	
	/********************************************//**
	 *  
	 * \brief build a update message for fluents
	 * 
	 * @param fname (string) the name of the fluent
	 * 
	 * @param fvalue (float) the new value for the fluent
	 * 
	 * @returns a copy of the message <i>rosplan_knowledge_msgs::KnowledgeUpdateService</i>,
	 * with the <i>.request</i> field ready for the service request. 
	 * 
	 * @todo implement params for fluents
	 * 
	 ***********************************************/
	rosplan_knowledge_msgs::KnowledgeUpdateService request_update(
		const std::string fname, 
		std::map<std::string, std::string>& params, 
		float fvalue );
	
	/********************************************//**
	 *  
	 * \brief search a particular fluent inside a list of fluents
	 * 
	 * Brute force search inside a list of fluents: the function looks for
	 * a fluent with that name and those arguments, and returns its value. 
	 * 
	 * @param fname the name of the fluent
	 * @param params the list of parameters
	 * @param res the response from the service containing the functionals
	 * 
	 * @returns the value of the fluent, if it exists; otherwise, see \ref ok
	 * 
	 * @note remember to check the validity of the result using \ref ok
	 * 
	 * @todo alternatives to a pure brute force search?
	 * 
	 ***********************************************/
	float read_fluents_list(
		const std::string fname, 
		std::map<std::string, std::string>& params,
		rosplan_knowledge_msgs::GetAttributeService::Response& res );
	
	/********************************************//**
	 *  
	 * \brief cast a KeyValue message into a simple map
	 * 
	 * this function is often useful, since the messages of the
	 * knowledge base use a lot the message type diagnostic_msgs::KeyValue. 
	 * 
	 * @param kv the vector of messages diagnostic_msgs::KeyValue
	 * 
	 * @returns a map from that vector
	 * 
	 ***********************************************/
	std::map<std::string, std::string> keyvalue2map( 
		const std::vector<diagnostic_msgs::KeyValue>& kv );

private:
	
	/// predicates query client handle
	ros::ServiceClient cl_query;
	
	/// another query client handle
	ros::ServiceClient cl_query_2;
	
	/// update clent handle
	ros::ServiceClient cl_kb_update;
	
	/// fluents query client handle
	ros::ServiceClient cl_kb_get_fluent;
	
	/********************************************//**
	 *  
	 * \brief open the services with the knowledge base
	 * 
	 * here is the list of the services opened by this function: 
	 * <ul>
	 * <li> \ref SERVICE_QUERY : rosplan_knowledge_msgs::KnowledgeQueryService> </li>
	 * <li> \ref SERVICE_KB_UPDATE : rosplan_knowledge_msgs::KnowledgeUpdateService </li>
	 * <li> ??? : ??? </li>
	 * </ul>
	 * 
	 ***********************************************/
	void open_services( );
};





#endif

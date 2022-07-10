
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
 * \brief more services for the robocluedo project!
 * 
 * ... more details
 * 
 * @see kb_tools
 * 
 ***********************************************/
class robocluedo_kb_tools : public kb_tools
{
public:
	
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
};

#endif

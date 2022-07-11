cmake_minimum_required(VERSION 3.0.2)
project(robocluedo_rosplan_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  std_msgs
  std_srvs
  diagnostic_msgs
  rosplan_dispatch_msgs
  rosplan_knowledge_msgs
  rosplan_dependencies
  roslib
  rostest
  robocluedo_rosplan_interface_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
	knowledge_base_tools 
	dispatch_actions 
	robocluedo_rosplan_action_interface
 CATKIN_DEPENDS 
	roscpp 
	actionlib 
	std_msgs 
	std_srvs 
	diagnostic_msgs
	rosplan_dispatch_msgs 
	rosplan_knowledge_msgs 
	roslib 
	rostest
#  DEPENDS system_lib
)

find_package(FLEX REQUIRED)

# Disable deprecated declarations warning (about std::auto_ptr)
add_definitions(-Wno-deprecated-declarations)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/robocluedo_rosplan_interface.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/robocluedo_rosplan_interface_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

## === LIBRARIES === ##

# knowledge_base_tools

add_library( kb_tools src/knowledge_base_tools/kb_tools.cpp )
target_link_libraries( kb_tools ${catkin_LIBRARIES} )

add_library( robocluedo_kb_tools src/knowledge_base_tools/robocluedo_kb_tools.cpp )
target_link_libraries( robocluedo_kb_tools ${catkin_LIBRARIES} kb_tools )

# action feedback manager

add_library( action_feedback_manager src/dispatch_actions/feedback_manager.cpp )
target_link_libraries( action_feedback_manager ${catkin_LIBRARIES} )

# dispatch actions

add_library( init_planning_system src/dispatch_actions/init_planning_system.cpp )
target_link_libraries( init_planning_system ${catkin_LIBRARIES} kb_tools robocluedo_kb_tools action_feedback_manager )

add_library( move_to src/dispatch_actions/move_to.cpp )
target_link_libraries( move_to ${catkin_LIBRARIES} kb_tools robocluedo_kb_tools action_feedback_manager )

add_library( manipulator_near_marker src/dispatch_actions/manipulator_near_marker.cpp )
target_link_libraries( manipulator_near_marker ${catkin_LIBRARIES} kb_tools robocluedo_kb_tools action_feedback_manager )

add_library( acquire_hint src/dispatch_actions/acquire_hint.cpp )
target_link_libraries( acquire_hint ${catkin_LIBRARIES} kb_tools robocluedo_kb_tools action_feedback_manager )

add_library( sherlock_is_thinking src/dispatch_actions/sherlock_is_thinking.cpp )
target_link_libraries( sherlock_is_thinking ${catkin_LIBRARIES} kb_tools robocluedo_kb_tools action_feedback_manager )

# rosplan action interface

add_library( robocluedo_rosplan_action_interface src/ActionInterface/RPActionInterface.cpp )
add_dependencies( robocluedo_rosplan_action_interface ${catkin_EXPORTED_TARGETS} )

## === CPP NODES === ##

# testing modules

add_executable( test_load_and_run src/testing/test_load_and_run.cpp )
add_dependencies( test_load_and_run ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( test_load_and_run ${catkin_LIBRARIES} )

add_executable( test_pipeline src/testing/test_pipeline.cpp )
add_dependencies( test_pipeline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( test_pipeline ${catkin_LIBRARIES} )

add_executable( test_pipeline_dispatch src/testing/test_pipeline_dispatch.cpp )
add_dependencies( test_pipeline_dispatch ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( test_pipeline_dispatch ${catkin_LIBRARIES} )

add_executable( test_kb_tools src/testing/test_kb_tools.cpp )
add_dependencies( test_kb_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( test_kb_tools ${catkin_LIBRARIES} kb_tools )

add_executable( test_robocluedo_kb_tools src/testing/test_robocluedo_kb_tools.cpp )
add_dependencies( test_robocluedo_kb_tools ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( test_robocluedo_kb_tools ${catkin_LIBRARIES} kb_tools robocluedo_kb_tools )

# dispatch actions

add_executable( node_init_planning_system src/node_init_planning_system.cpp )
add_dependencies( node_init_planning_system ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( node_init_planning_system ${catkin_LIBRARIES} kb_tools robocluedo_kb_tools robocluedo_rosplan_action_interface action_feedback_manager init_planning_system )

add_executable( node_move_to src/node_move_to.cpp )
add_dependencies( node_move_to ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( node_move_to ${catkin_LIBRARIES} kb_tools robocluedo_kb_tools robocluedo_rosplan_action_interface action_feedback_manager move_to )

add_executable( node_manipulator_near_marker src/node_manipulator_near_marker.cpp )
add_dependencies( node_manipulator_near_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( node_manipulator_near_marker ${catkin_LIBRARIES} kb_tools robocluedo_kb_tools robocluedo_rosplan_action_interface action_feedback_manager manipulator_near_marker )

add_executable( node_acquire_hint src/node_acquire_hint.cpp )
add_dependencies( node_acquire_hint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( node_acquire_hint ${catkin_LIBRARIES} kb_tools robocluedo_kb_tools robocluedo_rosplan_action_interface action_feedback_manager acquire_hint )

add_executable( node_sherlock_is_thinking src/node_sherlock_is_thinking.cpp )
add_dependencies( node_sherlock_is_thinking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( node_sherlock_is_thinking ${catkin_LIBRARIES} kb_tools robocluedo_kb_tools robocluedo_rosplan_action_interface action_feedback_manager sherlock_is_thinking )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robocluedo_rosplan_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

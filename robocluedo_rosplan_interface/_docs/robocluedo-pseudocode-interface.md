robocluedo-pseudocode-interface

# robocluedo ROS Plan -- Interfaces

these interfaces are implemented across many levels:

- the node `robocluedo_rosplan_kb_interface`  provides the ROS interface to communicate with the Knowledge Base of ROS plan

- the node above-mentioned uses a class `robocluedo_kb_tools` which implements some specific services for interacting with the knowledge base

- the class `robocluedo_kb_tools` inherits from another class `kb_tools` which implements the basic functionalities

## The main idea -- kb_tools

the knowledge base is seen as a container of predicates and numerical values. The project requires these primitives:

- GET/SET predicates
- GET/SET fluents
- REPLAN base method

### Constructor of kb_tools

the class assumes that a knowledge base is running at the construction time. The class creates the services and the topic subscriptions. 

## The main idea -- robocluedo_kb_tools

the class provides custom functionalities for the project *robocluedo*. They are:

- SETUP the ontology (first setup by default)
- REPLAN custom for robocluedo

custom functionalities:

- allocate hints
- set loops (eventually with a rule)

the main idea is that the class keeps a copy of the status of the ontology synchronized with the real ontology. It provides methods aimed at simplifying the access to the ontology as much as possible, with reference to the specific context of the project. 

Known elements by the class:

- names of predicates and functions

the class also implements a sort of **shared memory** between the actions: one action can communicate to another one which outcome is expected, for example. 

## Examples -- kb_tools


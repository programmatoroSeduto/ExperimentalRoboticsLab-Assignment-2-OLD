# Robocluedo -- Planning Procedure

## Work Elements

here are some elements of the architecture to take into account during the implementation. 

### navigation system

Available nodes of the system:

- `navigation_control` implements the navigation system with path planning. It desn't control the robotic manipulator
- `manipulator_control` implements a MoveIt controller for the manipulator. 

### how the Oracle works

here are some words about the expected behaviour by the oracle:

- (publication of the markers, not needed at this level of the problem)
- the oracle publishes a message each time the cluedo_link is near enough to a given marker
- there's a service enabling the application to check if the result is true

## preliminary list of actions

- `(move-to ?from ?to - place)` : move from place `?from` to the place `?to`
	
	it calls the services of the motion controller, in particular the `navigation_control` node.

- `(manipulator-near-hintpoint ?here - place ?wp - hintpoint)` : put the manipulator end effector near the hint point

- `(acquire-hint ?p - place ?wp - hintpoint)` : check for a hint from the hint point

- `(check-hint ?p - place ?wp - hintpoint)` : check if the received hint is malformed or not; the planner will assume that everything worked fine, but the real implementation uses this action to verify that the hint has been correctly received, and in case, the real implementation could require here a replanning. 

- `(manipulator-far-hintpoint ?here - place ?wp - hintpoint)` : put the manipulator tip far from the hint point

- `(move-to-center ?from - place)` : move the robot towards the center from the actual position, preparing to send the solution

- `(send-solution ?id - hyp-id)` : send one solution ID to the Oracle; if the hypothesis is correct, the plan ends, otherwise a replanning action is necessary; the external system will create a problem instance in which the ID tried here is discarded. 
	
	the principle is that the planner can freely choose a valid ID amond the valid ones, instead of manually selecting the ID to send.
	
	**note that** a ID should have been allocated, which means that a given ID has at least three different elements associated. For this reason, a action `(init-planner )` is required in order to setup the knowledge base according to the actual state. 

### How robocluedo works -- PDDL expected behaviour

Here's the expected solution from the problem:

1. starting from the center of the arena, move towards one of the waypoint using `move-to`
2. **LOOP FOR MAX 3 TIMES**:
	1. reach the waypoint, then call `manipulator-near-waypoint` to put the manipulator tip near to the hint point
	2. acquire a hint from the system using the action `acquire-hint`
	3. NO (check if the hint is valid using `check-hint`) NO
	4. call `manipulator-far-waypoint`
3. move in center of the arena using `move-to-center`
4. send the solution to the oracle: *the plan ends*

### How robocluedo works -- under the hood

here's the complete pseudocode. the rows starting with `(*)` are referred to the external planning system. the pseudocode also contains the action calls performed by the planner. 

```
(init-planning-system )
	# CONDITION : system not initialized
	(*) check hints status and allocate
	(*) set (max-loop ) function to the number of acquisitions to be performed
	# EFFECT : the system is working

# LOOP for (max-loop ) times
(move-to ?from ?to)
	# CONDITIONS
	# ?to is unexplored (adn near to ?from)
	# (max-loop ) > 0
	(*) -> navigation_controller : go to that marker
	# EFFECT : inside a unexplored point, and
(manipulator-near-hintpoint ?place ?wp)
	# CONDITIONS 
	# the position contains that waypoint
	# place is unexplored
	(*) -> manipulator_controller -> go near the marker
		# the node implementing the check on the hint receives the message from the topic
(acquire-hint ?place ?wp)
	# the hint has beed received before, and stored
	(*) checkk if the hint is valid or not
		# see the oracle node to understand what is a "not valid hint"
	(*) update the KB (if needed)
		# update the counter for that hypothesis only if the hint is really new
		# if really new, allocate the hints in the KB
		# update the problem status
(check-hint ?place ?wp)
	(*) check if the problem is still solvable
		# that means: at tle last loop, one of these is true
			# there's at least one consistent hypothesis to test
			#    OR
			# there's just one remaining consistent hypothesis
		# IF NOT SOLVABLE IN TIME : repplanning!
(manipulator-far-hintpoint ?place ?wp)
	# ready to move again

# END LOOP -- go on
(move-to-center ?place)
	# CONDITIONS: 
	# the robot is not in the center of the arena
	# and the robot is ready to send something
	(*) -> navigation_controller : go in the middle of the arena
(send-solution ?id)
	# CONDITIONS:
	# the robot is in the middle of the area
	# the robot is ready to send a ID
	# the ID should be consistent (the counter is equal to 3)
	(*) ->oracle : ask for the solution ID
	(*) IF the ID is correct : END OF THE PROBLEM
	(*) ELSE : !!replanning!!
	# the plan is concluded: the robot can do nothing after one solution has been sent
```

Symbols:

- `(action ...)` the PDDL action
- `#...` annnotation referred to the previously called action
- `(*)...` action performed from the external planning system

### How robocluedo works -- plan setup

in order to be able to plan, the planner will make some assumptions:

- ...

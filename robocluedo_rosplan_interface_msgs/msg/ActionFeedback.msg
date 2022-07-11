# message 'ActionFeedback'

## === about the action sending the feedback message

# feedback from action
string action_name
# with parameters
diagnostc_msgs/KeyValue[ ] parameters

## === plan statistics

# inconsistency
bool kb_not_consistent
# problem solvable (false: not solvable)
bool problem_solvable
# problem solvable by exclusion
bool by_exclusion

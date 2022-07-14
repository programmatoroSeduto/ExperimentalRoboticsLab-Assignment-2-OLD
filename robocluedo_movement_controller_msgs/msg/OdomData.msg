## message 'OdomData'
#    2D localisation informations, on demand

# the current position
geometry_msgs/Point position

# the frame name
string frame_name

# the message contains a distance
bool sent_distance 

# the target point
geometry_msgs/Point target

# the distance between the target and the current position
float32 distance

# the message contains a distance vector
# bool sent_distance_vector

# the distance vector between the target and the current position
# geometry_msgs/Vector3 distance_vector

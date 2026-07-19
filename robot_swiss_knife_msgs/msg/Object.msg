##############################################################################################################
# Message definition adapted from https://github.com/b-it-bots/mas_perception_msgs/blob/noetic/msg/Object.msg
##############################################################################################################

# The pose where the object can be found
geometry_msgs/PoseStamped pose

# The dimensions of the object
geometry_msgs/Vector3Stamped dimensions

# Unique name of the object determined e.g. by the recognition
string name

# Categorization of this object
string category

# The object's ID in a database
int32 database_id

# Probability
float32 probability

# View representation including an RGB image of the object and a point cloud,
# An array since we may have multiple views of the same object
robot_swiss_knife_msgs/ObjectView[] views

# 2D bounding box in an image
sensor_msgs/RegionOfInterest roi
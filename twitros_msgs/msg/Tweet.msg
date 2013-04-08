# Tweet ROS message (also used for direct message)

uint64 id	# Can be used to reply
time stamp	# Time of tweet

string user	# screen name of user that tweeted (@...)
string name	# true name of user that tweeted
string text	# The 140 char max tweet

sensor_msgs/Image image	# Optional

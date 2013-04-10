# Tweet ROS message (also used for direct message)

uint64 id	# ID of the tweet. Can be used to reply
time stamp	# Time of tweet (from twitter, only seconds are set)

string user	# screen name of user that tweeted (@...)
string name	# true name of user that tweeted
string text	# The 140 char max tweet

sensor_msgs/Image image	# Optional (if provided)

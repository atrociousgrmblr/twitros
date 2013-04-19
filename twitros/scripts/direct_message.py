#!/usr/bin/env python

import sys

import roslib; roslib.load_manifest('twitros')
import rospy

import cv
from cv_bridge import CvBridge, CvBridgeError

from twitros_msgs.srv import *

if __name__ == '__main__' :
    if len( sys.argv ) > 2:
        user = sys.argv[1]
        msg = sys.argv[2]
	paths = []
        for i in range(3, len( sys.argv )):
            paths.append(sys.argv[i])
    else:
        print("Usage: " + sys.argv[0] 
	    + ' SCREEN_NAME "MESSAGE" PATH_IMAGE_1 PATH_IMAGE_2 ...')
        sys.exit(1)

    rospy.wait_for_service('post_dm')
    direct_message = rospy.ServiceProxy('post_dm', DirectMessage)
    bridge = CvBridge()
   
    # Convert pics to sensor_msgs/Image
    images = []
    for path in paths:
        try:
            images.append( bridge.cv_to_imgmsg( cv.LoadImage(path) ) )    
        except CvBridgeError, e:
    	    rospy.logerr( "CvBridge error: " + e )

    try:
        direct_message( user = user, text = msg, images = images )
    except rospy.ServiceException, e:
        rospy.logerr( "Service call failed: " + e )

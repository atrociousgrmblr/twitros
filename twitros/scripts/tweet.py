#!/usr/bin/env python

import sys

import roslib; roslib.load_manifest('twitros')
import rospy

import cv
from cv_bridge import CvBridge, CvBridgeError

from twitros_msgs.srv import *

if __name__ == '__main__' :
    if len( sys.argv ) > 1:
        status = sys.argv[1]
	paths = []
        for i in range(2, len( sys.argv )):
            paths.append(sys.argv[i])
    else:
        print("Usage: " + sys.argv[0] 
	    + ' "MESSAGE" PATH_IMAGE_1 PATH_IMAGE_2 ...')
        sys.exit(1)

    rospy.wait_for_service('post_tweet')
    tweet = rospy.ServiceProxy('post_tweet', Post)
    bridge = CvBridge()
   
    # Convert pics to sensor_msgs/Image
    images = []
    for path in paths:
        try:
            images.append( bridge.cv_to_imgmsg( cv.LoadImage(path) ) )
        except CvBridgeError, e:
    	    rospy.logerr( "CvBridge error: " + e )
	    
    try:
        tweet( text = status, images = images )
    except rospy.ServiceException, e:
        rospy.logerr( e )

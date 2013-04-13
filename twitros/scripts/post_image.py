#!/usr/bin/env python

# Small image posting test

import sys

import roslib; roslib.load_manifest('twitros')
import rospy

import cv
from cv_bridge import CvBridge, CvBridgeError

from twitros_msgs.srv import *

if __name__ == '__main__' :
    if len( sys.argv ) == 2:
        path = sys.argv[1]
    else:
        sys.exit(1)

    rospy.wait_for_service('post_tweet')
    tweet = rospy.ServiceProxy('post_tweet', Post)
    bridge = CvBridge()
    
    status = 'Image posting test'
    
    print 'Openning ' + path
    cv_image = cv.LoadImage(path)    

    try:
        pic = bridge.cv_to_imgmsg( cv_image )
    except CvBridgeError, e:
    	print 'CvBridge error: ' + e

    try:
        tweet( text = status, image = pic )
    except rospy.ServiceException, e:
        print 'Service call failed.'

#!/usr/bin/env python
#
# Copyright 2013, IntRoLab, Universite de Sherbrooke.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

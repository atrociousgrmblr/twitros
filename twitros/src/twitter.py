#!/usr/bin/python2.7
#
# Copyright 2013, IntRoLab, Université de Sherbrooke.
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

import os
import sys

# ROS
import roslib
roslib.load_manifest('twitros')

# Twitter API from python_twitter
import twitter

# OAUth
try:
  from urlparse import parse_qsl
except:
  from cgi import parse_qsl

import oauth2 as oauth

REQUEST_TOKEN_URL = 'https://api.twitter.com/oauth/request_token'
ACCESS_TOKEN_URL  = 'https://api.twitter.com/oauth/access_token'
AUTHORIZATION_URL = 'https://api.twitter.com/oauth/authorize'
SIGNIN_URL        = 'https://api.twitter.com/oauth/authenticate'

# To manage pictures
import urllib
import cv
from cv_bridge import CvBridge, CvBridgeError

# Messages and services
from twitros_msgs.msg import *
from twitros_msgs.srv import *

class Twitter:
    def _init_(self):
    	# Publish mentions and timeline
        self.pub_mentions = rospy.Publisher("mentions", Tweets)
        self.pub_timeline = rospy.Publisher("timeline", Tweets)
        self.pub_dm = rospy.Publisher("direct_messages", Tweets)

	# Advertise services
	self.post = rospy.Service('post_tweet', Post, self.post_cb)
	self.retweet = rospy.Service('retweet', Post, self.retweet_cb)
	self.post_dm = rospy.Service('post_dm', DirectMessage, self.post_dm_cb)
	self.destroy = rospy.Service('destroy_dm', Destroy, self.destroy_cb)
	self.reply = rospy.Service('reply_tweet', Reply, self.reply_cb)

	# Process timer
	self.timer = rospy.Timer(rospy.Duration(2), self.timer_cb)
	    
	oauth_token_key = None
	oauth_token_secret = None

	# Get OAuth info through parameter server
	if rospy.has_param('token_key') and rospy.has_param('token_secret'):
	    oauth_token_key = rospy.get_param('token_key')
	    oauth_token_secret = rospy.get_param('token_secret')
	# OAuth token creation (see get_access_token.py from python-twitter)
	else:
	    print ''
	    print 'No OAuth information given, trying to create some...'
	    print ''

	    signature_method_hmac_sha1 = oauth.SignatureMethod_HMAC_SHA1()
	    oauth_consumer = oauth.Consumer(key = 'HbAfkrfiw0s7Es4TVrpSuw', 
	    		secret = 'oIjEOsEbHprUa7EOi3Mo8rNBdQlHjTGPEpGrItZj8c')
	    oauth_client = oauth.Client(oauth_consumer)

            # Request token
	    resp, content = oauth_client.request(REQUEST_TOKEN_URL, 'GET')
	    
	    if resp['status'] != '200':
  	        print 'Invalid respond from Twitter requesting temp token: %s'
	                                                   % resp['status']
            else:
                request_token = dict(parse_qsl(content))

	        print ''
		print 'Please visit this Twitter page and retrieve the'
		print 'pincode to be used in the next step to obtaining an '
		print 'Authentication Token:'
		print ''
		print '%s?oauth_token=%s' 
		    % (AUTHORIZATION_URL, request_token['oauth_token'])
		print ''

	        pincode = raw_input('Pincode? ')

  		token = oauth.Token(request_token['oauth_token'], 
				request_token['oauth_token_secret'])
  		token.set_verifier(pincode)

		print ''
  		print 'Generating and signing request for an access token'
  		print ''

  		oauth_client  = oauth.Client(oauth_consumer, token)
  		resp, content = oauth_client.request( ACCESS_TOKEN_URL, 
			method='POST', 
			body='oauth_callback=oob&oauth_verifier=%s' % pincode)
  		access_token  = dict(parse_qsl(content))

		# Parse
  		if resp['status'] != '200':
    		    print 'The request for a Token did not succeed: %s' 
		    					% resp['status']
    		    print access_token
  		else:
    		    print 'Your Twitter Access Token key: %s' 
		    				% access_token['oauth_token']
    		    print '             Access Token secret: %s' 
		    			% access_token['oauth_token_secret']
    		    print ''

            	    oauth_token_key = access_token['oauth_token']
                    oauth_token_secret = access_token['oauth_token_secret']

		    # TODO: Write OAuth info somewhere.

	# Consumer key and secret are specific to this App.
	# Access token are given through OAuth for those consumer params
	self.api = twitter.Api(
	    consumer_key = 'HbAfkrfiw0s7Es4TVrpSuw',
            consumer_secret = 'oIjEOsEbHprUa7EOi3Mo8rNBdQlHjTGPEpGrItZj8c',
            access_token_key = oauth_token_key,
            access_token_secret = oauth_token_secret )

	# Create a bridge for images conversions
	self.bridge = CvBridge()

    # Services callbacks
    # TODO: Find a good way to detect failures. Post images
    def post_cb(self, req):
        result = self.api.PostUpdate(req.text)
	if result.GetId() > 0:
	    return True
	else
	    return False

    def retweet(self, req):
        result = self.api.PostRetweet(req.id)
	if result.GetId() > 0:
	    return True
	else
	    return False

    def post_dm_cb(self, req):
        result = self.api.PostDirectMessage(req.user, req.text)
        if result.GetId() > 0:
	    return True
	else
	    return False
    
    def destroy_cb(self, req):
        result = self.api.DestroyDirectMessage(req.id)
	if result.GetId() > 0:
	    return True
	else
	    return False
    
    def reply_cb(self, req):
        result = self.api.PostUpdate(req.text, req.reply_to_id)
	if result.GetId() > 0:
	    return True
	else
	    return False

    # Convert tweets from python-twitter structure to ROS structure
    # Args: statuses: An array of statuses returned by Api.GetXXX().
    def process_tweets(self, statuses)
        msg = Tweets()
	for s in statuses:
	    tweet = Tweet()
	    tweet.id = s.id
	    tweet.stamp = ros.Time( s.created_at )
	    tweet.user = s.user.screen_name
	    tweet.name = s.user.name
	    tweet.text = s.text
	    msg.tweets.append(tweet)
	return msg

    # Convert DM from python-twitter structure to ROS structure
    # Args: dm: An array of dm returned by Api.GetDirectMessages().
    # Note: No distinction between Tweets and DM in ROS structure.
    def process_tweets(self, dm)
        msg = Tweets()
	for s in statuses:
	    tweet = Tweet()
	    tweet.id = s.id
	    tweet.stamp = ros.Time( s.created_at )
	    tweet.user = s.sender_screen_name
	    tweet.name = self.api.GetUser(s.sender_id).name # Retrieve name
	    tweet.text = s.text

            for url in s.urls:
	        # Download image
	        r = urllib.URLopener()
		r.retrieve(url, '/tmp/pic.jpg')

		# Read/Load with OpenCV
		cv_image = cv.LoadImage('/tmp/pic.jpg')

		# COnvert to ROS message using CV bridge.
		try:
		    tweet.image = self.bridge.cv_to_imgmsg(cv_image, 
		                                  encoding="passthrough")
		except CvBridgeError, e:
      		    print e

	    msg.tweets.append(tweet)
	return msg

    # Retrieve updates in timeline, mentions, and direct messages
    def timer_cb(self, event):
        # Timeline
	statuses = self.api.GetUserTimeline()
	timeline_msg = self.process_tweets( statuses )
	if timeline_msg.tweets.empty() == False:
	    self.pub_timeline.publish( timeline_msg )
	
	# Mentions (exact same process)
	mentions = self.api.GetMentions()
	mentions_msg = self.process_tweets( mentions )
	if mentions_msg.tweets.empty() == False:
	    self.pub_mentions.publish( mentions_msg )

	# Direct messages
	dm = self.api.GetDirectMessages()
	dm_msg = self.process_tweets( dm )
	if dm_msg.tweets.empty() == False:
	    self.pub_dm.publish( dm_msg )

if __name__ == '__main__' :
    rospy.init_node('twitter', anonymous=True)
    twitter = Twitter()
    rospy.spin()

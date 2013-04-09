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

import os
import sys

# ROS
import roslib; roslib.load_manifest('twitros')
import rospy

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

class TwitterServer:
    def __init__(self):
    	# Publish mentions, timeline and direct messages
        self.pub_timeline = rospy.Publisher("timeline", Tweets, latch=True)
        self.pub_mentions = rospy.Publisher("mentions", Tweets, latch=True)
        self.pub_dm = rospy.Publisher("direct_messages", Tweets, latch=True)

	# Advertise services
	self.post = rospy.Service('post_tweet', Post, self.post_cb)
	self.retweet = rospy.Service('retweet', Id, self.retweet_cb)
	self.reply = rospy.Service('reply_tweet', Reply, self.reply_cb)

	self.follow = rospy.Service('follow', Id, self.follow_cb)
	self.unfollow = rospy.Service('unfollow', Id, self.unfollow_cb)
	
	self.post_dm = rospy.Service('post_dm', DirectMessage, self.post_dm_cb)
	self.destroy = rospy.Service('destroy_dm', Id, self.destroy_cb)

	# Last Tweets
	self.last_mention = 0
	self.last_timeline = 0
	self.last_dm = 0

	self.api = twitter.Api()
	oauth_token_key = None
	oauth_token_secret = None

	# Get OAuth info through parameter server
	if rospy.has_param('~token_key'):
	    oauth_token_key = rospy.get_param('~token_key')
	if rospy.has_param('~token_secret'):
	    oauth_token_secret = rospy.get_param('~token_secret')

	# OAuth token creation (see get_access_token.py from python-twitter)
	if oauth_token_key is None or oauth_token_secret is None:
	    rospy.loginfo('No OAuth information given, trying to create...')

	    signature_method_hmac_sha1 = oauth.SignatureMethod_HMAC_SHA1()
	    oauth_consumer = oauth.Consumer(key = 'HbAfkrfiw0s7Es4TVrpSuw', 
	    		secret = 'oIjEOsEbHprUa7EOi3Mo8rNBdQlHjTGPEpGrItZj8c')
	    oauth_client = oauth.Client(oauth_consumer)

            # Request token
	    resp, content = oauth_client.request(REQUEST_TOKEN_URL, 'GET')
	    
	    if resp['status'] != '200':
  	        rospy.logerr("Invalid respond from Twitter requesting temp "
			+ "token: {resp}".format( resp=resp['status'] ) )
		sys.exit()
            else:
                request_token = dict(parse_qsl(content))

	        rospy.loginfo("Please visit this Twitter page and retrieve the"
		    + " pincode to be used in the next step to obtaining an "
		    + "Authentication Token: \n{url}?oauth_token={token}".
			format( url=AUTHORIZATION_URL, 
				token=request_token['oauth_token']) )

	        pincode = raw_input('Pincode? ')

  		token = oauth.Token(request_token['oauth_token'], 
				request_token['oauth_token_secret'])
  		token.set_verifier(pincode)

  		rospy.loginfo('Generating and signing request for a token')

  		oauth_client  = oauth.Client(oauth_consumer, token)
  		resp, content = oauth_client.request( ACCESS_TOKEN_URL, 
			method='POST', 
			body='oauth_callback=oob&oauth_verifier=%s' % pincode)
  		access_token  = dict(parse_qsl(content))

  		if resp['status'] != '200':
    		    rospy.logerr("The request for a Token did not succeed:"
		    		+ "{e}".format( e = resp['status'] ))
		    sys.exit()
    		    #ros.loginfo( access_token )
  		else:
    		    rospy.loginfo("Your Twitter Access Token key: [{key}] ".
		    	    format( key=access_token['oauth_token'] )
		        + "and Access Token secret: [{secret}].".
		    	    format( secret=access_token['oauth_token_secret'] )
			+ "\nPlease copy those for future use.")

            	    oauth_token_key = access_token['oauth_token']
                    oauth_token_secret = access_token['oauth_token_secret']
		    # TODO: Write OAuth info somewhere.
	else:
		rospy.loginfo("Using the following parameters for oauth:"
		    + "key: {key}, ".format(key = oauth_token_key)
		    + "secret: {secret}".format(secret = oauth_token_secret))

	# Consumer key and secret are specific to this App.
	# Access token are given through OAuth for those consumer params
	rospy.loginfo('Trying to log API...')
	self.api = twitter.Api(
	    consumer_key = 'HbAfkrfiw0s7Es4TVrpSuw',
            consumer_secret = 'oIjEOsEbHprUa7EOi3Mo8rNBdQlHjTGPEpGrItZj8c',
            access_token_key = oauth_token_key,
            access_token_secret = oauth_token_secret )
	
	result = self.api.VerifyCredentials()
	rospy.loginfo('Twitter connected as {user}!'
				.format(user = result.screen_name))

	# Create a bridge for images conversions
	self.bridge = CvBridge()
	
	# Create timer for tweet retrieval
	self.timer = rospy.Timer(rospy.Duration(15), self.timer_cb)

    # Services callbacks
    # TODO: Find a good way to detect failures. Post images.
    def post_cb(self, req):
        result = self.api.PostUpdate(req.text)
	return PostResponse()

    def retweet_cb(self, req):
        result = self.api.PostRetweet(req.id)
	return IdResponse()
    
    def follow_cb(self, req):
        result = self.api.CreateFrienship(req.id)
	return IdResponse()
    
    def unfollow_cb(self, req):
        result = self.api.DestroyFrienship(req.id)
	return IdResponse()

    def post_dm_cb(self, req):
        result = self.api.PostDirectMessage(req.user, req.text)
	return DirectMessageResponse()
    
    def destroy_cb(self, req):
        result = self.api.DestroyDirectMessage(req.id)
	return IdResponse()
    
    def reply_cb(self, req):
        result = self.api.PostUpdate(req.text, req.reply_to_id)
	return ReplyResponse()

    # Convert tweets from python-twitter structure to ROS structure
    # Args: statuses: An array of statuses returned by Api.GetXXX().
    def process_tweets(self, statuses):
        msg = Tweets()
	for s in statuses:
	    tweet = Tweet()
	    tweet.id = s.id
	    tweet.stamp = rospy.Time( s.created_at_in_seconds )
	    tweet.user = s.user.screen_name
	    tweet.name = s.user.name
	    tweet.text = s.text

	    if s.urls is not None:
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
      		        rospy.logerr(e)

	    msg.tweets.append(tweet)
	return msg

    # Convert DM from python-twitter structure to ROS structure
    # Args: dm: An array of dm returned by Api.GetDirectMessages().
    # Note: No distinction between Tweets and DM in ROS structure.
    def process_dm(self, dm):
        msg = Tweets()
	for m in dm:
	    tweet = Tweet()
	    tweet.id = m.id
	    tweet.stamp = rospy.Time( m.created_at_in_seconds )
	    tweet.user = m.sender_screen_name
	    tweet.name = self.api.GetUser(m.sender_id).name # Retrieve name
	    tweet.text = m.text
	    msg.tweets.append(tweet)

	return msg

    # Retrieve updates in timeline, mentions, and direct messages
    def timer_cb(self, event):
        # Timeline
	statuses = self.api.GetFriendsTimeline( since_id = self.last_timeline )
	timeline_msg = self.process_tweets( statuses )
	if len(timeline_msg.tweets):
	    # Copy id of last tweet
	    self.last_timeline = timeline_msg.tweets[0].id
	    self.pub_timeline.publish( timeline_msg )
	
	# Mentions (exact same process)
	mentions = self.api.GetMentions( since_id = self.last_mention )
	mentions_msg = self.process_tweets( mentions )
	if len(mentions_msg.tweets):
	    self.last_mention = mentions_msg.tweets[0].id
	    self.pub_mentions.publish( mentions_msg )

	# Direct messages
	dm = self.api.GetDirectMessages( since_id = self.last_dm )
	dm_msg = self.process_dm( dm )
	if len(dm_msg.tweets):
	    self.last_dm = dm_msg.tweets[0].id
	    self.pub_dm.publish( dm_msg )

if __name__ == '__main__' :
    rospy.init_node('twitter_server')
    twitter_server = TwitterServer()
    rospy.spin()

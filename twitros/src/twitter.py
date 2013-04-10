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

# TODO: Need to work on rates limit.

import os
import sys

# ROS
import roslib; roslib.load_manifest('twitros')
import rospy

# Twitter API from python_twitter
from twitter import *

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

	self.follow = rospy.Service('follow', User, self.follow_cb)
	self.unfollow = rospy.Service('unfollow', User, self.unfollow_cb)
	
	self.post_dm = rospy.Service('post_dm', DirectMessage, self.post_dm_cb)
	self.destroy = rospy.Service('destroy_dm', Id, self.destroy_cb)

	# Last Tweets
	self.last_mention = 0
	self.last_timeline = 0
	self.last_dm = 0

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

	    oauth_dance( "IntRoLab TwitROS", 'HbAfkrfiw0s7Es4TVrpSuw',
                'oIjEOsEbHprUa7EOi3Mo8rNBdQlHjTGPEpGrItZj8c', creds)

	    oauth_token_key, oauth_token_secret = read_token_file( creds )

	else:
	    rospy.loginfo("Using the following parameters for oauth:"
		    + "key: {key}, ".format(key = oauth_token_key)
		    + "secret: {secret}".format(secret = oauth_token_secret))

	# Consumer key and secret are specific to this App.
	# Access token are given through OAuth for those consumer params
	rospy.loginfo('Trying to log into Twitter API...')
	self.api = twitter( auth=OAuth(oauth_token_key, oauth_token_secret 
	   		'HbAfkrfiw0s7Es4TVrpSuw',
	   		'oIjEOsEbHprUa7EOi3Mo8rNBdQlHjTGPEpGrItZj8c'))
	
	result = self.api.account.verify_credentials()
	rospy.loginfo('Twitter connected as {name} (@{user})!'
		.format(name = result['name'], user = result['screen_name']))

	# Create a bridge for images conversions
	self.bridge = CvBridge()
	
	# Create timers for tweet retrieval
	self.timer_home = rospy.Timer(rospy.Duration(2), self.timer_home_cb)
	self.timer_mentions = rospy.Timer(rospy.Duration(2), 
						self.timer_mentions_cb)
	self.timer_dm = rospy.Timer(rospy.Duration(2), self.timer_dm_cb)

    # Services callbacks
    # TODO: Find a good way to detect failures. Post images.
    def post_cb(self, req):
        if (req.reply_id == 0):
            result = self.api.statuses.update( text = req.text )
	else:
            result = self.api.statuses.update( text = req.text,
	    			in_reply_to_status_id = req.reply_id )
	return PostResponse()

    def retweet_cb(self, req):
        result = self.api.statuses.retweet( id = req.id )
	return IdResponse()
    
    def follow_cb(self, req):
        result = self.api.friendships.create( screen_name = req.user )
	return UserResponse()
    
    def unfollow_cb(self, req):
        result = self.api.friendships.destroy( screen_name = req.user )
	return UserResponse()

    def post_dm_cb(self, req):
        result = self.api.direct_messages.new( 
			screen_name = req.user, text = req.text )
	return DirectMessageResponse()
    
    def destroy_cb(self, req):
        result = self.api.direct_messages.destroy( id = req.id )
	return IdResponse()
    
    # Convert tweets from twitter structure to ROS structure
    # Args: statuses: An array of statuses returned by the api call.
    def process_tweets(self, statuses):
        msg = Tweets()
	for s in statuses:
	    tweet = Tweet()
	    tweet.id = s['id']
	    ts = time.strftime('%Y-%m-%d %H:%M:%S', 
	    	time.strptime(s['created_at'],'%a %b %d %H:%M:%S +0000 %Y'))
	    tweet.stamp = rospy.Time.from_sec( time.mktime(ts) )
	    tweet.user = s['user']['screen_name']
	    tweet.name = s['user']['name']
	    tweet.text = s['text']

	    for entities in s['entities']:
            	 if entities['type'] is "photo":
	            # Download image
	            r = urllib.URLopener()
		    r.retrieve(entities['media_url'], '/tmp/pic.jpg')

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
	    tweet.id = m['id']
	    ts = time.strftime('%Y-%m-%d %H:%M:%S', 
	    	time.strptime(m['created_at'],'%a %b %d %H:%M:%S +0000 %Y'))
	    tweet.stamp = rospy.Time.from_sec( time.mktime(ts) )
	    tweet.user = m['sender_screen_name']
	    tweet.name = m['sender']['name']
	    tweet.text = m['text']
	    msg.tweets.append(tweet)

	return msg

    # Retrieve updates in timeline, mentions, and direct messages
    def timer_home_cb(self, event):
        # Timeline: To be changed when 1.1 is on.
	#statuses = self.api.GetHomeTimeline( since_id = self.last_timeline )
	statuses = self.api.statuses.home_timeline( 
				since_id = self.last_timeline )
	timeline_msg = self.process_tweets( statuses )
	if len(timeline_msg.tweets):
	    # Copy id of last tweet
	    self.last_timeline = timeline_msg.tweets[0].id
	    self.pub_timeline.publish( timeline_msg )
	
	# Reset tiler to stick to rate limits 
	# See: https://dev.twitter.com/docs/rate-limiting/1.1/limits
	del self.timer_home
	response = self.api.application.rate_limit_status(
					ressources = "statuses" )
	frame = rospy.Duration( rospy.Time.from_sec( 
	    response['/statuses/home_timeline']['reset']) - rospy.Time.now())
	period = rospy.Duration.from_sec( frame.to_sec() 
	    / response['/statuses/home_timeline']['remaining'] )
	self.timer_home = rospy.Timer( period, self.timer_home_cb )

	
    def timer_mentions_cb(self, event):
	# Mentions (exact same process)
	mentions = self.api.statuses.mentions_timeline( 
					since_id = self.last_mention )
	mentions_msg = self.process_tweets( mentions )
	if len(mentions_msg.tweets):
	    self.last_mention = mentions_msg.tweets[0].id
	    self.pub_mentions.publish( mentions_msg )
	
	# Update rate
	del self.timer_mentions
	response = self.api.application.rate_limit_status(
					ressources = "statuses" )
	frame = rospy.Duration( rospy.Time.from_sec( 
	    response['/statuses/mentions_timeline']['reset']) 
	        - rospy.Time.now())
	period = rospy.Duration.from_sec( frame.to_sec() 
	    / response['/statuses/mentions_timeline']['remaining'] )
	self.timer_mentions = rospy.Timer( period, self.timer_mentions_cb )

    def timer_dm_cb(self, event):
	# Direct messages (make sure to not skip status)
	dm = self.api.direct_messages( since_id = self.last_dm, 
					skip_status = False )
	dm_msg = self.process_dm( dm )
	if len(dm_msg.tweets):
	    self.last_dm = dm_msg.tweets[0].id
	    self.pub_dm.publish( dm_msg )
	
	# Update rate
	del self.timer_dm
	response = self.api.application.rate_limit_status(
					resources = "direct_messages" )
	frame = rospy.Duration( rospy.Time.from_sec( 
	    response['/direct_messages']['reset']) - rospy.Time.now() )
	period = rospy.Duration.from_sec( 
	    frame.to_sec() / response['direct_messages']['remaining'] )
	self.timer_mentions = rospy.Timer( period, self.timer_mentions_cb )

if __name__ == '__main__' :
    rospy.init_node('twitter_server')
    twitter_server = TwitterServer()
    rospy.spin()

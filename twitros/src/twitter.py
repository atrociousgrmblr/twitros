#!/usr/bin/env python

import os
import sys
from time import time, mktime, strftime, strptime

# ROS
import roslib; roslib.load_manifest('twitros')
import rospy

# Twitter API from pwython 
from twython import Twython

# To manage pictures
import urllib
import cv
from cv_bridge import CvBridge, CvBridgeError

# Messages and services
from twitros_msgs.msg import *
from twitros_msgs.srv import *

class TwitterServer:
    def __init__(self):

        # You can choose to latch tweets
	if rospy.has_param('latch'):
	    latch = rospy.get_param('latch')
	else:
	    latch = False
    	
	# Publish mentions, timeline and direct messages
        self.pub_timeline = rospy.Publisher('timeline', Tweets, latch = latch)
        self.pub_mentions = rospy.Publisher('mentions', Tweets, latch = latch)
        self.pub_dm = rospy.Publisher('direct_messages', Tweets, latch = latch)
	
	# Create a bridge for images conversions
	self.bridge = CvBridge()

	# Last Tweets (init values are twitter API default)
	self.last_mention = 12345
	self.last_timeline = 12345
	self.last_dm = 12345

	oauth_token = None
	oauth_token_secret = None

	# Get OAuth info through parameter server
	if rospy.has_param('~token'):
	    oauth_token = rospy.get_param('~token')
	if rospy.has_param('~token_secret'):
	    oauth_token_secret = rospy.get_param('~token_secret')

	# OAuth token creation (see get_access_token.py from python-twitter)
	if oauth_token is None or oauth_token_secret is None:
	    rospy.loginfo('No OAuth information given, trying to create...')

	    t = Twython( app_key =  'HbAfkrfiw0s7Es4TVrpSuw',
                app_secret = 'oIjEOsEbHprUa7EOi3Mo8rNBdQlHjTGPEpGrItZj8c')

	    auth_props = t.get_authentication_tokens()
	    oauth_token = auth_props['oauth_token']
	    oauth_token_secret = auth_props['oauth_token_secret']

	    del t

	rospy.loginfo('Using the following parameters for oauth: '
		    + 'key: [{key}], '.format(key = oauth_token)
		    + 'secret: [{secret}]'.format(secret = oauth_token_secret))

	# Consumer key and secret are specific to this App.
	# Access token are given through OAuth for those consumer params
	rospy.loginfo('Trying to log into Twitter API...')

	# Twython
	self.t = Twython(app_key = 'HbAfkrfiw0s7Es4TVrpSuw',
            app_secret = 'oIjEOsEbHprUa7EOi3Mo8rNBdQlHjTGPEpGrItZj8c',
            oauth_token = oauth_token,
            oauth_token_secret = oauth_token_secret)
	
	result = self.t.verifyCredentials();
	rospy.loginfo('Twitter connected as {name} (@{user})!'
		.format(name = result['name'], user = result['screen_name']))

	# Advertise services
	self.post = rospy.Service('post_tweet', Post, self.post_cb)
	self.retweet = rospy.Service('retweet', Id, self.retweet_cb)

	self.follow = rospy.Service('follow', User, self.follow_cb)
	self.unfollow = rospy.Service('unfollow', User, self.unfollow_cb)
	
	self.post_dm = rospy.Service('post_dm', DirectMessage, self.post_dm_cb)
	self.destroy = rospy.Service('destroy_dm', Id, self.destroy_cb)
	
	# Create timers for tweet retrieval
	timer_home = rospy.Timer(rospy.Duration(1), self.timer_home_cb, 
							oneshot = True )
	timer_mentions = rospy.Timer(rospy.Duration(2), 
				self.timer_mentions_cb, oneshot = True )
	timer_dm = rospy.Timer(rospy.Duration(3), self.timer_dm_cb,
							oneshot = True )

    # Services callbacks
    def post_cb(self, req):
        # Handle image
    	if (req.image.height != 0):
	    # Convert from ROS message using cv_bridge.
            try:
	        cv_image = self.bridge.imgmsg_to_cv( req.image, 
		                        desired_encoding = 'passthrough')
	    except CvBridgeError, e:
      		rospy.logerr(e)

	    # Write to JPG with OpenCV
	    path = "/tmp/pic_ul_{time}.png".format( time = time() )
	    cv.SaveImage( path, cv_image)
            
            if (req.reply_id == 0):
                r = self.twython.updateStatusWithMedia( file_ = path, 
	    		status = req.text )
	    else:
                r = self.twython.updateStatusWithMedia( file_ = path, 
	    		status = req.text, in_reply_status_id = req.reply_id )
	
        elif (req.reply_id == 0):
            result = self.twython.updateStatus( status = req.text )
	else:
            result = self.twython.updateStatus( status = req.text,
	    			in_reply_to_status_id = req.reply_id )
	return PostResponse()

    def retweet_cb(self, req):
        result = self.t.retweet( id = req.id )
	return IdResponse()
    
    def follow_cb(self, req):
        result = self.t.createFriendship( screen_name = req.user )
	return UserResponse()
    
    def unfollow_cb(self, req):
        result = self.t.destroyFriendship( screen_name = req.user )
	return UserResponse()

    def post_dm_cb(self, req):
        result = self.t.sendDirectMessage( 
			screen_name = req.user, text = req.text )
	return DirectMessageResponse()
    
    def destroy_cb(self, req):
        result = self.t.destroyDirectMessage( id = req.id )
	return IdResponse()
    
    # Convert tweets from twitter structure to ROS structure
    # Args: statuses: An array of statuses returned by the api call.
    def process_tweets(self, statuses):
        msg = Tweets()
	for s in statuses:
	    tweet = Tweet()
	    tweet.id = s['id']
	    tweet.user = s['user']['screen_name']
	    tweet.name = s['user']['name']
	    tweet.text = s['text']
	    
	    # Convert twitter time string to ros Time structure
	    ts = strptime( s['created_at'],'%a %b %d %H:%M:%S +0000 %Y' )
	    tweet.stamp = rospy.Time.from_sec( mktime(ts) )

            # Handle image. Download, load with OpenCV and convert to ROS
	    if 'media' in s['entities']:
	        for media in s['entities']['media']:
            	    if media['type'] == 'photo':
			rospy.logdebug('Image detected: ' + media['media_url'])
			# Download image
	                r = urllib.URLopener()
			path = '/tmp/pic_{num}_{time}.{ext}'.format( 
				num = tweet.id, time = time(),
				ext = media['media_url'].split('.')[-1] )
		        r.retrieve( media['media_url'], path );

		        # Read/Load with OpenCV
		        cv_image = cv.LoadImage(path)
		        #os.system('rm -vf ' + path)

		        # Convert to ROS message using CV bridge.
		        try:
		            tweet.image = self.bridge.cv_to_imgmsg(cv_image, 
		                                  encoding='passthrough')
			    break
		        except CvBridgeError, e:
      		            rospy.logerr(e)

	    msg.tweets.append(tweet)
	return msg

    # Convert DM from python-twitter structure to ROS structure
    # Args: dm: An array of dm returned by getDirectMessages().
    # Note: No distinction between Tweets and DM in ROS structure.
    def process_dm(self, dm):
        msg = Tweets()
	for m in dm:
	    tweet = Tweet()
	    tweet.id = m['id']
	    tweet.user = m['sender_screen_name']
	    tweet.name = m['sender']['name']
	    tweet.text = m['text']
	    
	    # Time conversion
	    ts = strptime( m['created_at'],'%a %b %d %H:%M:%S +0000 %Y' )
	    tweet.stamp = rospy.Time.from_sec( mktime(ts) )
	    
	    msg.tweets.append(tweet)

	return msg

    # Retrieve updates in timeline, mentions, and direct messages
    def timer_home_cb(self, event):
        # Timeline: To be changed when 1.1 is on.
	response = self.t.getHomeTimeline( since_id = self.last_timeline, 
						include_entities = True )
	timeline_msg = self.process_tweets( response )
	if len(timeline_msg.tweets):
	    # Copy id of last tweet
	    self.last_timeline = timeline_msg.tweets[0].id
	    self.pub_timeline.publish( timeline_msg )
	
	# Reset timer to stick to rate limits 
	# See: https://dev.twitter.com/docs/rate-limiting/1.1/limits
	remaining = int( self.t.get_lastfunction_header( 
						'x-rate-limit-remaining' ))
	reset = int( self.t.get_lastfunction_header('x-rate-limit-reset') )
	
	if not remaining:
	    rospy.logwarn("Twitter: Limit reached for timeline. Sleeping...")
	    rospy.sleep( rospy.Duration( reset - rospy.Time.now().secs ) )
	    timer_home = rospy.Timer( rospy.Duration(1), self.timer_home_cb,
	    						oneshot = True )
	else:    
	    period = rospy.Duration.from_sec( ( reset - rospy.Time.now().secs )
	    						/ remaining )
	    timer_home = rospy.Timer( period, self.timer_home_cb, 
	    						oneshot = True )
	
    def timer_mentions_cb(self, event):
	# Mentions (exact same process)
	response = self.t.getMentionsTimeline( since_id = self.last_mention,
						include_entities = True )
	mentions_msg = self.process_tweets( response )
	if len(mentions_msg.tweets):
	    self.last_mention = mentions_msg.tweets[0].id
	    self.pub_mentions.publish( mentions_msg )
	
	remaining = int( self.t.get_lastfunction_header( 
					'x-rate-limit-remaining' ) )
	reset = int( self.t.get_lastfunction_header('x-rate-limit-reset') )
	
	# Update rate
	if not remaining:
	    rospy.logwarn("Twitter: Limit reached for mentions. Sleeping...")
	    rospy.sleep( rospy.Duration( reset - rospy.Time.now().secs ) )
	    timer_mentions = rospy.Timer( rospy.Duration(1),
	    			self.timer_mentions_cb, oneshot = True )
	else:
	    period = rospy.Duration.from_sec( ( reset - rospy.Time.now().secs )
	    						/ remaining )
	    timer_mentions = rospy.Timer( period, self.timer_mentions_cb,
	    						oneshot = True )
	    #rospy.loginfo("Twitter: rem: {rem}, next: {per}".format(
		#rem = response.rate_limit_remaining, per = period.to_sec()) )

    def timer_dm_cb(self, event):
	# Direct messages (make sure to not skip status)
	response = self.t.getDirectMessages( since_id = self.last_dm, 
					skip_status = False )
	dm_msg = self.process_dm( response )
	if len(dm_msg.tweets):
	    self.last_dm = dm_msg.tweets[0].id
	    self.pub_dm.publish( dm_msg )
	
	remaining = int( self.t.get_lastfunction_header( 
						'x-rate-limit-remaining' ) )
	reset = int( self.t.get_lastfunction_header( 'x-rate-limit-reset' ) )
	
	# Update rate
	if not remaining:
	    rospy.logwarn(
	        "Twitter: Limit reached for direct messages. Sleeping...")
	    rospy.sleep( rospy.Duration( reset - rospy.Time.now().secs ) )
	    timer_dm = rospy.Timer( rospy.Duration(1), self.timer_dm_cb,
	    						oneshot = True)

        else:
	    period = rospy.Duration.from_sec( ( reset - rospy.Time.now().secs )
	    						/ remaining )
	    timer_dm = rospy.Timer( period, self.timer_dm_cb, oneshot = True)

if __name__ == '__main__' :
    rospy.init_node('twitter_server')
    twitter_server = TwitterServer()
    rospy.spin()

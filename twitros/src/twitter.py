#!/usr/bin/env python

import os
import sys
from time import time, mktime, strftime, strptime

# ROS
import roslib; roslib.load_manifest('twitros')
import rospy

# Twitter API from twython 
from twython import Twython

# To manage pictures
import urllib
import cv
from cv_bridge import CvBridge, CvBridgeError

# Messages and services for twitros
from twitros_msgs.msg import *
from twitros_msgs.srv import *

class TwitterServer:
    def __init__(self):

        # You can choose to latch tweets topics
	latch = False
	if rospy.has_param('latch'):
	    latch = rospy.get_param('latch')

	# In case you can't direct message a user, replace DM with a public
	#	'@user text' tweet.
	replace_dm = False
	if rospy.has_param('replace_dm'):
	    replace_dm = rospy.get_param('replace_dm')
    	
	# Publish mentions, home timeline and direct messages
        self.pub_home = rospy.Publisher('home_timeline', Tweets, latch = latch)
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
	
	    # Get AUth URL. Use for login for security, 
	    # screen_name can be on rosparam
	    if rospy.has_param('~screen_name'):
	    	sn = rospy.get_param('~screen_name')
	    	url = t.get_authentication_tokens( 
	    		screen_name = sn, force_login = True)
	    else:
	    	url = t.get_authentication_tokens( force_login = True )

	    # Open web browser on given url
	    import webbrowser
	    webbrowser.open( url['auth_url'] )

	    # Enter pincode
	    pincode = raw_input('Pincode: ')

	    auth_props = t.get_authorized_tokens( oauth_verifier = int(pincode) )

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

	# Stock screen name (used to show friendships)
	self.name = result['screen_name']

	# Advertise services
	self.post = rospy.Service('post_tweet', Post, self.post_cb)
	self.retweet = rospy.Service('retweet', Id, self.retweet_cb)

	self.follow = rospy.Service('follow', User, self.follow_cb)
	self.unfollow = rospy.Service('unfollow', User, self.unfollow_cb)
	
	self.post_dm = rospy.Service('post_dm', DirectMessage, self.post_dm_cb)
	self.destroy = rospy.Service('destroy_dm', Id, self.destroy_cb)

	self.timeline = rospy.Service('user_timeline', 
					Timeline, self.user_timeline_cb)
	
	# Create timers for tweet retrieval
	timer_home = rospy.Timer(rospy.Duration(1), self.timer_home_cb, 
							oneshot = True )
	timer_mentions = rospy.Timer(rospy.Duration(2), 
				self.timer_mentions_cb, oneshot = True )
	timer_dm = rospy.Timer(rospy.Duration(3), self.timer_dm_cb,
							oneshot = True )

    # Tweet callback
    def post_cb(self, req):
	rospy.logdebug('Received a tweet: ' + req.txt)
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
	    os.system('rm -f ' + path)
            
            if (req.reply_id == 0):
                result = self.t.updateStatusWithMedia( file_ = path, 
	    		status = req.text )
	    else:
                result = self.t.updateStatusWithMedia( file_ = path, 
	    		status = req.text, in_reply_status_id = req.reply_id )
	
        elif (req.reply_id == 0):
            result = self.t.updateStatus( status = req.text )
	else:
            result = self.t.updateStatus( status = req.text,
	    			in_reply_to_status_id = req.reply_id )
	
	# Return false if call failed
	if self.t.get_lastfunction_header('status') != '200 OK':
	    return None
	else: 
	    return PostResponse(id = result['id'])

    def retweet_cb(self, req):
        result = self.t.retweet( id = req.id )
	if self.t.get_lastfunction_header('status') != '200 OK':
	    return None
	else: 
	    return IdResponse()
    
    # Does not raise an error if you are already following the user
    def follow_cb(self, req):
	rospy.logdebug('Asked to follow:' + req.user)
        result = self.t.createFriendship( screen_name = req.user )
	if self.t.get_lastfunction_header('status') != '200 OK':
	    return None
	else: 
	    return UserResponse()
    
    # Does not raise an error if you are not following the user
    def unfollow_cb(self, req):
	rospy.logdebug('Asked to unfollow:' + req.user)
        result = self.t.destroyFriendship( screen_name = req.user )
	if self.t.get_lastfunction_header('status') != '200 OK':
	    return None
	else: 
	    return UserResponse()

    # Send direct message.
    def post_dm_cb(self, req):
	rospy.logdebug('Received DM to ' + req.user + ': ' + req.text)
        
	# First, check if you can dm the user
	relation = self.t.showFriendship( 
		source_screen_name = self.name, target_screen_name = req.user )
	
	if self.t.get_lastfunction_header('status') != '200 OK':
	    rospy.logerr('Failed to get friendship information.')
	    return None

	# If he can, send a direct message
	if relation['source']['can-dm'] == 'true':
            result = self.t.sendDirectMessage( 
			screen_name = req.user, text = req.text )
    # If he cant but was allowed to send a tweet instead, tweet with mention
	elif replace_dm:
	    rospy.logwarn("You can't send a direct message to " + req.user 
	    	+ ". Sending a public tweet...")
            result = self.t.updateStatus( 
	    		status = "@" + req.user + " " + req.text )
	else:
	    rospy.logwarn("You can't send a direct message to " + req.user)
	    return None

	if self.t.get_lastfunction_header('status') != '200 OK':
	    return None
	else:
	    return DirectMessageResponse(id = result['id'])
    
    def destroy_cb(self, req):
        result = self.t.destroyDirectMessage( id = req.id )

	if self.t.get_lastfunction_header('status') != '200 OK':
	    return None
	else: 
	    return IdResponse()

    def user_timeline_cb(self, req):
        result = self.t.getUserTimeline( screen_name = req.user )
	if self.t.get_lastfunction_header('status') != '200 OK':
	    return None
	else: 
	    msg = self.process_tweets( result )
	    if msg:
	        return TimelineResponse( tweets = msg )
	    else:
	        rospy.logwarn(req.user + ' has no tweets in its timeline.')
	        return TimelineResponse( )
    
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
		        os.system('rm -f ' + path)

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
	    self.pub_home.publish( timeline_msg )
	
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

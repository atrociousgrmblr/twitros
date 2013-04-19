#!/usr/bin/env python

import os
from time import time, mktime, strptime

# ROS
import roslib; roslib.load_manifest('twitros')
import rospy

# Twitter API from twython 
from twython import Twython

# To manage pictures
import cv
from cv_bridge import CvBridge, CvBridgeError

# Messages and services for twitros
from twitros_msgs.msg import *
from twitros_msgs.srv import *

# Direct message image hosting
import requests
from bs4 import BeautifulSoup


class TwitterServer:
    def __init__(self):
        # You can choose to latch tweets topics
	latch = False
	if rospy.has_param('latch'):
	    latch = rospy.get_param('latch')

	# In case you can't direct message a user, replace DM with a public
	#	'@user text' tweet.
	self.replace_dm = False
	if rospy.has_param('replace_dm'):
	    self.replace_dm = rospy.get_param('replace_dm')
    	
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
	    rospy.loginfo("No OAuth information given, trying to create...")

	    t = Twython( app_key =  'HbAfkrfiw0s7Es4TVrpSuw',
                app_secret = 'oIjEOsEbHprUa7EOi3Mo8rNBdQlHjTGPEpGrItZj8c')
	
	    # Get AUth URL. Use for login for security, 
	    url = t.get_authentication_tokens( )

	    t = Twython(app_key = 'HbAfkrfiw0s7Es4TVrpSuw',
            	app_secret = 'oIjEOsEbHprUa7EOi3Mo8rNBdQlHjTGPEpGrItZj8c',
            	oauth_token = url['oauth_token'],
            	oauth_token_secret = url['oauth_token_secret'])

	    # Open web browser on given url
	    import webbrowser
	    webbrowser.open( url['auth_url'] )
	    
	    # Wait to avoid webbrowser to corrupt raw_input
	    rospy.sleep( rospy.Duration( 5 ) )

	    # Enter pincode
	    pincode = raw_input('Pincode: ').strip()

	    auth_props = t.get_authorized_tokens(oauth_verifier = pincode)

	    oauth_token = auth_props['oauth_token']
	    oauth_token_secret = auth_props['oauth_token_secret']

	    del t

	rospy.loginfo("Using the following parameters for oauth: "
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
	rospy.loginfo("Twitter connected as {name} (@{user})!"
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
	txt = req.text
	rospy.logdebug("Received a tweet: " + txt)

        # If only one picture, use twitter upload
    	if len(req.images) == 1:
	    path = self.save_image( req.images[0] )
            
            if (req.reply_id == 0):
                result = self.t.updateStatusWithMedia( file_ = path, 
	    		status = req.text )
	    else:
                result = self.t.updateStatusWithMedia( file_ = path, 
	    		status = req.text, in_reply_status_id = req.reply_id )
	    
	    os.system('rm -f ' + path)
	
	elif len(req.images) != 0:
	    txt +=  upload( req.images )
	
        if (req.reply_id == 0):
            result = self.t.updateStatus( status = txt )
	else:
            result = self.t.updateStatus( status = txt,
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
	rospy.logdebug("Asked to follow:" + req.user)
        result = self.t.createFriendship( screen_name = req.user )
	if self.t.get_lastfunction_header('status') != '200 OK':
	    return None
	else: 
	    return UserResponse()
    
    # Does not raise an error if you are not following the user
    def unfollow_cb(self, req):
	rospy.logdebug("Asked to unfollow:" + req.user)
        result = self.t.destroyFriendship( screen_name = req.user )
	if self.t.get_lastfunction_header('status') != '200 OK':
	    return None
	else: 
	    return UserResponse()

    # Send direct message.
    def post_dm_cb(self, req):
	rospy.logdebug("Received DM to " + req.user + ": " + req.text)
        
	# First, check if you can dm the user
	relation = self.t.showFriendship( 
		source_screen_name = self.name, target_screen_name = req.user )
	
	if self.t.get_lastfunction_header('status') != '200 OK':
	    rospy.logerr("Failed to get friendship information.")
	    return None

	# If he can, send a direct message
	if relation['relationship']['source']['can_dm']:
	    txt = req.text
	    # Upload image to postimage.org using requests
	    if len(req.images) != 0:
		txt += self.upload( req.images ) 
            result = self.t.sendDirectMessage( 
			screen_name = req.user, text = txt )

        # If he cant but was allowed to send a tweet instead, tweet with mention
	elif self.replace_dm:
	    rospy.logwarn("You can't send a direct message to " + req.user 
	    	+ ". Sending a public tweet...")
	    # One image ---> Twitter
	    if len(req.images) == 1 :
	        path = self.save_image( req.images[0] )
                result = self.t.updateStatusWithMedia( file_ = path, 
	    		status = req.text )
                os.system('rm -rf ' + path)
	    else:
	    	status = '@' + req.user + ' ' + req.text
		# Many images ---> postimage.org
	        if len(req.images) != 0:
		    status +=  upload( req.images )
                result = self.t.updateStatus( status = status )
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
	        rospy.logwarn(req.user + " has no tweets in its timeline.")
	        return TimelineResponse( )

    # Upload array of sensor_msgs/Image to postimage.org and return link.
    # Link is shortened if possible.
    def upload(self, images):
        files = {}
	paths = [] # Keep paths stored to remove files later
	# Construct files dict
	i = 0
	for image in images:
	    paths.append( self.save_image( image ) )
	    files['upload[{count}]'.format(count = i)] = open(paths[i], 'rb')
	    i += 1
	
	# Post using requests
	request = requests.post('http://postimage.org/index.php', 
		files = files, params = {'optsize': 0, 'adult': 'no'})

	# Cleanup saved files
	for path in paths:
	    os.system('rm -rf ' + path)

	if not request.status_code in [301, 201, 200]:
	    return " [FAILED UPLOAD]"

	# Parse HTML page returned using beautifulsoup
	soup = BeautifulSoup(request.text, 'html.parser')

	# Find the image link: I hacked that by looking at raw html file
	url = ""
	if len(images) == 1:
	    # 1 image : find first solo link containg 'postimg'
	    for link in soup.find_all('a'):
		if link.get('href').find('postimg') != -1:
		    url = link.get('href')
		    break
	else:
	    # Many images: find the option field for gallery url
	    for option in soup.find_all('option'):
		if option.get('value').find('gallery') != -1:
		    url = option.get('value')
		    break
	
	if not len(url):
	    return " [FAILED PARSING OR UPLOAD]"
		
	# Shorten URL
	request = requests.get( 'http://is.gd/create.php',
	     params = {'format': 'simple', 'url': url} )

	if request.status_code in [301, 201, 200]:
	    return "  [" + request.text + "]"
	else:
	    return "  [" + url + "]"
    
    # Save a sensor_msgs/Image on /tmp and return the path
    def save_image(self, image):
        # Convert from ROS message using cv_bridge.
        try:
	    cv_image = self.bridge.imgmsg_to_cv( image, 
				desired_encoding = 'passthrough')
        except CvBridgeError, e:
	    rospy.logerr(e)

        # Write to JPG with OpenCV
        path = "/tmp/pic_ul_{time}.png".format( time = time() )
        cv.SaveImage( path, cv_image )
	return path

    
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
			rospy.logdebug("Image detected: " + media['media_url'])

			# Download and write image using requests
			path = '/tmp/pic_{num}_{time}.{ext}'.format( 
				num = tweet.id, time = time(),
				ext = media['media_url'].split('.')[-1] )
			f = open(path, 'wb')
			f.write( requests.get(media['media_url']).content )
			f.close()

		        # Read/Load with OpenCV
		        cv_image = cv.LoadImage( path )
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
	# Get headers just after function to avoid getting another call result
	remaining = int( self.t.get_lastfunction_header( 
						'x-rate-limit-remaining' ))
	reset = int( self.t.get_lastfunction_header('x-rate-limit-reset') )

	timeline_msg = self.process_tweets( response )
	if len(timeline_msg.tweets):
	    # Copy id of last tweet
	    self.last_timeline = timeline_msg.tweets[0].id
	    self.pub_home.publish( timeline_msg )
	
	# Reset timer to stick to rate limits 
	# See: https://dev.twitter.com/docs/rate-limiting/1.1/limits
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
	
	remaining = int( self.t.get_lastfunction_header( 
					'x-rate-limit-remaining' ) )
	reset = int( self.t.get_lastfunction_header('x-rate-limit-reset') )

	mentions_msg = self.process_tweets( response )
	if len(mentions_msg.tweets):
	    self.last_mention = mentions_msg.tweets[0].id
	    self.pub_mentions.publish( mentions_msg )
	
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
	
	remaining = int( self.t.get_lastfunction_header( 
						'x-rate-limit-remaining' ) )
	reset = int( self.t.get_lastfunction_header( 'x-rate-limit-reset' ) )

	dm_msg = self.process_dm( response )
	if len(dm_msg.tweets):
	    self.last_dm = dm_msg.tweets[0].id
	    self.pub_dm.publish( dm_msg )
	
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

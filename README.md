TwitROS
=======

[Twitter] [8] driver for [ROS] [2] in python.
Implemented using [twython] [1]. 
Developped at [IntRoLab] [4].

Please note that this project in still in its early stage of creation and 
is unstable at the moment.

Dependencies
---

TwitROS uses [twython] [1] as its twitter library:

    (pip install | easy_install) twython beautifulsoup4

Installation
---

After cloning, you can build twitros by running:

    rosmake twitros

Don't forget to make the python files runable:
    
    roscd twitros
    chmod +x src/twitter.py

Running the Twitter server
---

We provide a sample launchfile you can run. In this one, OAuth ROS 
parameters are not set so you will be prompted to create a token.
You can then copy the generated token and use it later through `rosparam`
(see next section).

    roslaunch twitros twitter.launch
    
If OAuth is a success, you should see your name appears under the form:
<pre>Twitter connected as NAME (@SCREEN_NAME)!</pre>

Using OAuth
---

Once you have your token you can use it by setting the `token` 
and `token_secret` parameters:

<pre><code>&lt;launch>
    &lt;node name="twitros" pkg="twitros" type="twitter.py" output="screen">
        &lt;rosparam name="token" value="PUT_KEY_HERE"/>
        &lt;rosparam name="token_secret" value="PUT_SECRET_HERE"/>
	&lt;/node>
&lt;/launch></code></pre>

Services provided
---

The driver provides the following services at the moment:

* `post_tweet`: Post a tweet. You can also mark it as a reply. Return the
tweet id. See section about image posting.
* `retweet`: Retweet a tweet given its id.
* `follow`: Follow a user.
* `unfollow`: Unfollow a user.
* `post_dm`: Send a direct message to a user. Return an id. 
See section about image posting.
* `destroy_dm`: Destroy a direct message given its id.
* `user_timeline`: Return the timeline of a user (array of tweets).

You can get more info by reading the services in `twitros_msgs/srv` folder.

Topics published
---

New tweets are retrieved at a variable rate depending on [API limits] [5].
Current rate limits allow a message to be published every minute 
approximatively.
Each of the following bullets represents a topic that publishs 
`twitros_msgs/Tweets` messages.
The server succesfully retrieves images (saving to `/tmp` then deleting) 
and converts them to `sensors_msgs/Image` using [OpenCV] [6] 
and [cv_bridge] [7].

* `home_timeline`: tweets from your timeline.
* `mentions`: tweets that mention you.
* `direct_messages`: your direct messages. *Note:* Tweet and Direct message 
use the same `twitros_msgs/Tweet` message structure.

You can get more info by reading the messages in `twitros_msgs/msg` folder.

ROS parameters
---
Parameters you can set:

* `latch`: Topics latch tweets. (default: <code>false</code>).
* `replace_dm`: If it's not possible to send a direct message, replace by
send a public tweet under the form:
<pre>@USER TEXT</pre> 
It can be useful since you can't send a direct message to someone not 
following you. (default: <code>false</code>).

Script
---
Some test scripts are given in `twitros/scripts/`.

* `tweet.py`: Post a tweet with images
* `direct_message.py`: Post a tweet with images

Images posting
---
Twitter support only image posting in tweets and only one image can be 
joined per tweets. `twitros` enables to post multiple images indirectly 
by hosting them on [postimage] [9]. [postimage] [9] is always used when
you add image to a `twitros_msgs/DirectMessage` request and add the
gallery url at the end of the tweet. On `twitros_msgs/Post` requests,
it will use [postimage] [9] only if there is more than one.

[1]: https://github.com/ryanmcgrath/twython "Twython"
[2]: http://ros.org "ROS"
[3]: http://www.pip-installer.org "pip"
[4]: http://introlab.3it.usherbrooke.ca "Introlab"
[5]: https://dev.twitter.com/docs/rate-limiting/1.1 "Twitter rate limiting"
[6]: http://opencv.willowgarage.com/documentation/python/reading_and_writing_images_and_video.html "OpenCV python Load/Save"
[7]: https://ros.org/wiki/cv_bridge "cv_bridge wiki"
[8]: http://twitter.com "Twitter"
[9]: http://postimage.org "PostImage, free image hosting."

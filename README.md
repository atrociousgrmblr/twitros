twitros
=======

Twitter for [ROS] [2] using new JSON API (version 1.1).
Implemented using [python-twitter] [1]. 
Developped at [IntRoLab] [4].

Please note that this project in still in its early stage of creation and 
is unstable at the moment.

Installation
---

After cloning, you can build twitros by running:

    sudo apt-get install python-pip
    rosmake rostweet

You will need to enter your login informations during `python_twitter` 
building since it uses [pip] [3] to install some external libraries 
(see [python-twitter] [1] for more details).

Don't forget to make the python files runable:
    
    roscd twitros
    chmod +x src/twitter.py

Introduction
---

This projects creates a driver node that provides various twitter services 
and publish on topics the tweets you receive.

Running the server
---

We provide a sample launchfile you can run. In this one, OAuth ROS 
parameters are not set so you will be prompted to create a token.
You will need to copy the URL and paste it in your browser, connect
to your twitter account, authorize twitros and copy the given pincode
in the 
You can then copy the generated token and use it later through `rosparam`
(see next section).

    roslaunch twitros twitter.launch
    
If OAuth is a success, you should see your twitter username appears.
<pre>Twitter connected as USERNAME!</pre>

Using OAuth
---

Once you have your token you can use it by setting the `token_key` 
and `token_secret` parameters:

<pre><code>&lt;launch>
    &lt;node name="twitros" pkg="twitros" type="twitter.py">
        &lt;rosparam name="token_key" value="PUT_KEY_HERE"/>
        &lt;rosparam name="token_secret" value="PUT_SECRET_HERE"/>
	&lt;/node>
&lt;/launch></code></pre>

Services provided
---

The driver provides the following services at the moment:

* `post_tweet`: Post a tweet.
* `reply`: Post a tweet as a reply to another tweet.
* `retweet`: Retweet a tweet given its id.
* `post_dm`: Send a direct message to a user.
* `destroy_dm`: Destroy a direct message given its id.

You can get more info by reading the services in `twitros_msgs/srv` folder.

Topics published
---

New tweets are retrieved at a rate given in parameter (default is 2 seconds).
Each of the following bullets represents a topic that publishs 
`twitros_msgs/Tweets` messages.

* `timeline`: tweets from your timeline.
* `mentions`: tweets that mention you.
* `direct_messages`: your direct messages. *Note:* Tweet and Direct message 
use the same `twitros_msgs/Tweet` message structure.

You can get more info by reading the messages in `twitros_msgs/msg` folder.

TODO
---
* [python-twitter] [1] can do a lot more than proposed here like searching 
or rerieving tweets from others user. More functunalities could be 
implemented in the future.
* Better handling of images (post and retrieve).

Contact
---

You can contact me at tronche.adrien@gmail.com

[1]: https://github.com/bear/python-twitter "python-twitter"
[2]: http://ros.org "ROS"
[3]: http://www.pip-installer.org "pip"
[4]: http://introlab.3it.usherbrooke.ca "Introlab"

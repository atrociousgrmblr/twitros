twitros
=======

Twitter for [ROS] [2] using new JSON API (version 1.1). Implemented using [python-twitter] [1]. Developped at [IntRoLab] [4].

Please note that this project in still in its early stage of creation and is unstable at the moment.

Installation
---

After cloning, you can build twitros by running:

    rosmake rostweet

You will need to enter your login informations during `python_twitter` building since it uses 
[pip] [3] to install some external libraries (see [python-twitter] [1] for more details).

Introduction
---

This projects creates a driver node that provides various twitter services and publish the tweets you receive.

Services provided
---

The driver provides the following services at the moment:

* `post_tweet`: Post a tweet.
* `reply`: Post a tweet as a reply to another tweet.
* `retweet`: Retweet a tweet given its id.
* `post_dm`: Send a direct message to a user.
* `destroy_dm`: Destroy a direct message given its id.

You can get more info by reading the services from `rostweet_msgs/srv`.

Topics published
---

New tweets are retrieved at a rate given in parameter (default is 2 seconds)

* `timeline`: tweets from your timeline.
* `mentions`: tweets that mention you.
* `direct_messages`: your direct messages. Tweets and Direct message use the same `rostwit_msgs/msg/Tweet.msg` message structure.

You can get more info by reading the messages from `rostweet_msgs/msg`.

TODO
---
* [python-twitter] [1] can do a lot more than proposed here like searching or rerieving tweets from others user. 
More functunalities could be implemented in the future.
* Better handling of images (post and retrieve).

Contact
---

You can contact me at tronche.adrien@gmail.com

[1]: https://github.com/bear/python-twitter "python-twitter"
[2]: http://ros.org "ROS"
[3]: http://www.pip-installer.org "pip"
[4]: http://introlab.3it.usherbrooke.ca "Introlab"

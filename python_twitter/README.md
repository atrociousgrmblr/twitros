python_twitter
==============

This package is a ROS wrapper for the [python-twitter] [1] library.
It will compile and copy the python file in the `src` folder.

Building
---

You need to install `pip` before building:

    sudo apt-get install python-pip
    rosmake python_twitter

You will be prompted for your password when downloading external librairies
via pip.

[1]: https://github.com/bear/python-twitter "python-twitter"

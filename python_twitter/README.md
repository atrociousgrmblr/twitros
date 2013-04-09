python_twitter
==============

This package is a ROS wrapper for the [python-twitter] [1] library.
It will download the project, compile and copy the python file in 
the `src` folder. That way, it can easily be used by other ROS packages.

Building
---

You need to install `pip` before building:

    sudo apt-get install python-pip
    rosmake python_twitter

You will be prompted for your password when downloading external librairies
via pip.

Documentation
---

After building you can access the python documentation by typing 
`roscd python_twitter/src` and for examples:

    pydoc twitter.Api
    pydoc twitter.Status
    pydoc twitter.User

[1]: https://github.com/bear/python-twitter "python-twitter"

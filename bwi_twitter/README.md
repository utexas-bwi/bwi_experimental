UT Austin BWI Twitter Communications
===================================

This project is part of the continuing building-wide intergration project of the Computer Science Department in The University of Texas at Austin. This provides a framework to communicate with the Twitter API and allow the robots to perform actions and have conversations with Twitter users.

Packages
-----

* **twitcher_manager**
  * **twitcher_manager_node**: Manager is in charge of forwarding messages and deciding when to respond to tweets.
* **twitcher_connection**
  * **twitcher_connection_node**: This node is in charge of receiving and sending Tweets. It directly connects with the Twitter API.
* **twitcher_interpreter**
  * **twitcher_interpreter_node**: This node converts a given node into an action message, such as going to a certain room or finding a person.
* **twitcher_actions**
  * **twitcher_actions_node**: This node executes the actions send by the interpreter/manager node. It does this through actionlib servers.
* **twitcher_launch**: Contains launch files to run the necesary nodes for this module to work. Also contains configuration files.

Prerequisites and Dependencies
---
* GCC 4.9 or higher
* curlpp package

The `twitcher_interpreter` node requires makes use of the C++11 regex library, which was not implemented until GCC 4.9. For this reason, it must be installed and set as the default version of gcc. To do this, you must add the appropriate repository in apt, install it, and set it as the default version for both apt, which also sets the gcc alias. Here are the full set of commands:

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-4.9 g++-4.9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 60 --slave /usr/bin/g++ g++ /usr/bin/g++-4.8
```

This will ensure you can switch back to GCC 4.8 (for whatever reason you would have to do that). To switch back, all you have to do is run:

```bash
sudo update-alternatives --config gcc
```

This is based on the askubuntu answer: http://askubuntu.com/a/581497

To install curlpp:

```bash
sudo apt-get install libcurlpp-dev
```

Configuration Files
---

Inside `twitcher_launch` you can find the `config` directory with two yaml files: `config-template.yaml` and `rooms.yaml`. The latter contains
information on the rooms the robot can access (noticeably, it contains common names for the different rooms). The former file, `config-template.yaml`
contains a template of the file needed to configure the twitter API and connect with the appropriate cedentials. This file should be renamed 
`config.yaml` and filled in with the empty fields.

Future Work
---

* Add the option for a user to reply back (directly on the robot)
* Add information on who sent the tweet as well as the entire text
* Make `twitcher_connection` more robust (it currently fails if the message format is different from expected).
* Find alternate implementations of `twitcher_interpreter` that allow for any message format, instead of being constrained to a regex.
* Add responses to tweets (such as "I'm on my way!" or "I just got to room **LOCATION**")


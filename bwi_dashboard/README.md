bwi_dashboard
=================

Experimental web dashboard to control various components of a BWI robot.

##Dependencies

There are 3 main components that must be run in order for the application to work as intended:
- rosbridge
 - Used to communicate topics to and from the dashboard
 - Used in: Teleop, Navigation, Sensors
 - Run using: `roslaunch rosbridge_server rosbridge_websocket.launch`
- mjpeg_server
 - Used to stream video to the dashboard
 - Used in: Video
 - Run using: `rosrun mjpeg_server mjpeg_server __image_transport:=compressed`
- robot_pose_publisher
 - Used to stream location of robot
 - Used in: Navigation
 - Run using: `rosrun robot_pose_publisher robot_pose_publisher`


##How to use

There is a file, `js/config.js`, that contains default configurations that can be used with this program. However, you can also input connection and topic information on the configuration screen of the dashboard.

Open the index.html page in a browser. First input the proper configuration information, then click connect. When the dashboard displays, open the panels that you need. Note: for Teleop, the Q,W,E,A,S,D keys can be used for control as well.
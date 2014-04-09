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
 - Run using: `rosrun mjpeg_server mjpeg_server`
- robot_pose_publisher
 - Used to stream location of robot
 - Used in: Navigation
 - Run using: `rosrun robot_pose_publisher robot_pose_publisher`


##How to use

There is a file, `js/config.js`, that contains configuration information for the dashboard. The names should be self explanatory for what they represent. I am in the process of making this step easier and more straightforward to edit, but if you're running the browser on localhost, then the only things you should need to change are `tableSubscriptions` to hold the topics you are interested in streaming to Sensors and `rosMjpegServerInfo`'s `Topics` and `Labels` fields to hold the video topics you would like to stream to Video.

Open the index.html page in a browser and open the panels that you need.
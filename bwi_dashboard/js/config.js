//***************************************************************************
// EDIT VALUES BELOW

//The address/port of the running ROSBRIDGE Server
var rosBridgeAddress = 'ws://localhost:9090';

//The address of the runing Mjpeg Server
var rosMjpegVideoAddress = 'localhost';

//The port of the running Mjpeg Server
var rosMjpegVideoPort = 8080;

//The quality of the streamed video (from 1-100)
var rosMjpegVideoQuality = 10;

//The topics that will be viewable from video window
var rosMjpegVideoTopics = ['/nav_kinect/rgb/image_raw'];

//The subscriptions that are shown in the sensor table
//The format of the function is: createSubscriptionObject(topic name, topic type, topic filters)
//For instance, to stream just the linear and angular subtopics of odom, you would use:
//createSubscriptionObject('/odom', 'nav_msgs/Odometry', ['twist/twist/linear','twist/twist/angular'])
var tableSubscriptions = [
  createSubscriptionObject('/odom', 'nav_msgs/Odometry', ['twist/twist/linear','twist/twist/angular']),
  createSubscriptionObject('/clock', 'rosgraph_msgs/Clock', [])
]

// STOP EDITING VALUES
//***************************************************************************






var rosMjpegServerInfo = {
                           Host : rosMjpegVideoAddress,
                           Port : rosMjpegVideoPort,
                           Quality : rosMjpegVideoQuality, //0-100
                           Height : 480,
                           Width : 640,
                           Topics : rosMjpegVideoTopics,
                           Labels : rosMjpegVideoTopics
}

//The server name for the navigation window
var navWindowServerName = '/move_base';

//The topic name for teleop
var teleopTopic = '/cmd_vel';

//The key values to represent when the following button id is pressed (used for teleop)
//needed for CustomKeyboardTeleop.js
var buttonsToHandle = [{id: 'teleopLeftKey', val: 81},
                         {id: 'teleopUpKey', val: 87},
                         {id: 'teleopRightKey', val: 69},
                         {id: 'teleopTurnLeftKey', val: 65},
                         {id: 'teleopDownKey', val: 83},
                         {id: 'teleopTurnRightKey', val: 68}];


//Used to make the creation of a subscription object easy
function createSubscriptionObject(topicname, messagetype, subtopicnames)
{
 return {
         TopicName: topicname,
         MessageType: messagetype,
         SubTopicNames: subtopicnames
       };
}

//TO DO:
//Add variables to control which widgets are shown
//.htaccess file to make use controlled
//text window at top of screen to stream messages to user, like "You are logged onto X"
//compressed image transport (__image_transport:=compressed)
//Get parameters for topics etc from GET parameters
var connectionConfig = {
  "localhost" : createConnectionConfig("localhost", 9090, 8080),
  "hypnotoad" : createConnectionConfig("hypnotoad.csres.utexas.edu", 9090, 8080)
}

var subscriptionConfig = {
  "default" : {
              Enabled: {
                          Video:   true,
                          Teleop:  true,
                          Nav:     true,
                          Sensors: true
                        },
              VideoQuality: 10,
              VideoTopics: [
                            '/nav_kinect/rgb/image_raw'
                           ],
              SensorTopics: [
                              createSensorSubscriptionObject('/odom', 'nav_msgs/Odometry', ['twist/twist/linear', 'twist/twist/angular']),
                              createSensorSubscriptionObject('/clock', 'rosgraph_messages/clock', [])
                            ]

  },
  "default2" : {
              Enabled: {
                          Video:   false,
                          Teleop:  false,
                          Nav:     false,
                          Sensors: true
                        },
              VideoQuality: 0,
              VideoTopics: [
                           ],
              SensorTopics: [
                              createSensorSubscriptionObject('/odom', 'nav_msgs/Odometry', [])
                            ]
  }
}

//These topics are shown in the log at the top of the screen
//Add them in the form of createSensorSubscriptionObject(topicName, topicType, subtopics),
// where subtopics is an array of the specific values you are looking to extract from the topic, such as ['twist/twist/linear', 'twist/twist/angular']
var logTopics = [
                  // createSensorSubscriptionObject('/odom', 'nav_msgs/Odometry', [])
                ];


// STOP EDITING VALUES
//***************************************************************************

 //The server name for the navigation window
  var navWindowServerName = '/move_base';

  //The topic name for teleop
  var teleopTopic = '/cmd_vel';

  var rosMjpegServerInfo;

  //The address/port of the running ROSBRIDGE Server
  var rosBridgeAddress;

  //The address of the runing Mjpeg Server
  var rosMjpegVideoAddress;

  //The port of the running Mjpeg Server
  var rosMjpegVideoPort;

  //The quality of the streamed video (from 1-100)
  var rosMjpegVideoQuality = 10;

  //The topics that will be viewable from video window
  var rosMjpegVideoTopics;

  //Which widgets are enabled
  var enabledVideo;
  var enabledTeleop;
  var enabledNavigation;
  var enabledSensors;

  //The subscriptions that are shown in the sensor table
  //The format of the function is: createSubscriptionObject(topic name, topic type, topic filters)
  //For instance, to stream just the linear and angular subtopics of odom, you would use:
  //createSubscriptionObject('/odom', 'nav_msgs/Odometry', ['twist/twist/linear','twist/twist/angular'])
  var tableSubscriptions;

//The key values to represent when the following button id is pressed (used for teleop)
//needed for CustomKeyboardTeleop.js
var buttonsToHandle = [{id: 'teleopLeftKey', val: 81},
                         {id: 'teleopUpKey', val: 87},
                         {id: 'teleopRightKey', val: 69},
                         {id: 'teleopTurnLeftKey', val: 65},
                         {id: 'teleopDownKey', val: 83},
                         {id: 'teleopTurnRightKey', val: 68}];


//Used to make the creation of a subscription object easy
function createConnectionConfig(host, rosbridgeport, mjpegserverport)
{
 return {
         Host: host,
         RosbridgePort: rosbridgeport,
         MjpegServerPort: mjpegserverport
       };
}

//Used to make the creation of a sensor subscription object easy
function createSensorSubscriptionObject(topicname, messagetype, subtopicnames)
{
 return {
         TopicName: topicname,
         MessageType: messagetype,
         SubTopicNames: subtopicnames
       };
}

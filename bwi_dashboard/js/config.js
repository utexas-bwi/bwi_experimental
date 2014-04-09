var rosBridgeAddress = 'ws://localhost:9090';

var rosMjpegServerInfo = {
                           Host : 'localhost',
                           Port : 8080,
                           Quality : 100, //0-100
                           Height : 480,
                           Width : 640,
                           Topics : [
                                     '/nav_kinect/rgb/image_raw'
                                     ],
                           Labels : [
                                     '/nav_kinect/rgb/image_raw'
                                     ]
}

var navWindowServerName = '/move_base';

var tableSubscriptions = [

createSubscriptionObject('/odom', 'nav_msgs/Odometry', ['twist/twist/linear','twist/twist/angular']),
createSubscriptionObject('/clock', 'rosgraph_msgs/Clock', [])

]

var teleopTopic = '/cmd_vel';

//needed for CustomKeyboardTeleop.js
var buttonsToHandle = [{id: 'teleopLeftKey', val: 81},
                         {id: 'teleopUpKey', val: 87},
                         {id: 'teleopRightKey', val: 69},
                         {id: 'teleopTurnLeftKey', val: 65},
                         {id: 'teleopDownKey', val: 83},
                         {id: 'teleopTurnRightKey', val: 68}];



function createSubscriptionObject(topicname, messagetype, subtopicnames)
{
 return {
         TopicName: topicname,
         MessageType: messagetype,
         SubTopicNames: subtopicnames
       };
}


//Add variables to control which widgets are shown
//.htaccess file to make use controlled
//text window at top of screen to stream messages to user, like "You are logged onto X"
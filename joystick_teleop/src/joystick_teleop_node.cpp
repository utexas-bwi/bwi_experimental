#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

double MAX_SPEED_MULT = .5;
double MAX_TURN_MULT = .25;

bool firstZeroVel = false;
bool publishAnyway = false;

//Will hold values of sticks and buttons
int lastButtons[11];
float lastAxes[8];

//Updates the current values
void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  //Assign the values in a for loop (for some reason got a seg fault when just assigning vectors, prob
  //has to do with the pointer being deleted after the msg is over)
  int x;
  for(x = 0; x < 11; x++)
    lastButtons[x] = msg->buttons[x];
  
  int y;
  for(y = 0; y < 8; y++)
    lastAxes[y] = msg->axes[y];
  
  if(lastAxes[1] < 0.2 && lastAxes[1] > -0.2) //Dead zone for 0
    lastAxes[1] = 0;
  
  if(lastAxes[3] < 0.2 && lastAxes[3] > -0.2) //Dead zone for 0
     lastAxes[3] = 0;
  
  //Make sure we accept the 0 vel commands
  if(lastAxes[1] == 0 && lastAxes[3] == 0 && lastAxes[6] == 0 && lastAxes[7] == 0)
    firstZeroVel = true;
  
  //Up on D pad
  if(lastAxes[6] == -1)
  {
    MAX_TURN_MULT += .05;
    ROS_INFO("Turn Multiplier Changed: %f -> %f\n", MAX_TURN_MULT - .05, MAX_TURN_MULT);
  }
  
  //Down on D pad
  if(lastAxes[6] == 1)
  {
    MAX_TURN_MULT -= .05;
    ROS_INFO("Turn Multiplier Changed: %f -> %f\n", MAX_TURN_MULT + .05, MAX_TURN_MULT);
  }
  
  //Left on D pad
  if(lastAxes[7] == -1)
  {
    MAX_SPEED_MULT -= .1;
    ROS_INFO("Speed Multiplier Changed: %f -> %f\n", MAX_SPEED_MULT + .1, MAX_SPEED_MULT);
  }
  
  //Right on D pad
  if(lastAxes[7] == 1)
  {
    MAX_SPEED_MULT += .1;
    ROS_INFO("Speed Multiplier Changed: %f -> %f\n", MAX_SPEED_MULT - .1, MAX_SPEED_MULT);
  }
  
  //Emergency stop, sets the system to spam 0 vel commands until let go.
  //Red "B" button on xbox controllers
  if(lastButtons[1] == 1)
  {
    lastAxes[1] = 0;
    lastAxes[3] = 0;
    publishAnyway = true;
  }
  else
  {
    publishAnyway = false;
  }
}

int main(int argc, char **argv)
{
  
  if(!(argc == 1 || argc == 3))
    ROS_INFO("Invalid number of starting parameters, ignoring all of them.");
  
  //Can start with specific speed mults
  if(argc == 3)
  {
    MAX_SPEED_MULT = atof(argv[1]);
    MAX_TURN_MULT = atof(argv[2]);
    ROS_INFO("Starting with Speed Mult of %f, and Turn Mult of %f", MAX_SPEED_MULT, MAX_TURN_MULT);
  }
  
  //Set default
  if(MAX_SPEED_MULT == 0)
  {
    MAX_SPEED_MULT = .5;
    ROS_INFO("Adjusted Speed Mult to .5");
  }
  
  //Set default
  if(MAX_TURN_MULT == 0)
  {
    MAX_TURN_MULT = .25;
    ROS_INFO("Adjusted Turn Mult to .25");
  }
  
  ros::init(argc, argv, "joystick_teleop");

  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  
  ros::Subscriber sub = n.subscribe("joy", 1000, joystickCallback);
  
  ros::Rate loop_rate(10);
 
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    
    //Set forward/back based on left joystick up/down
    msg.linear.x = lastAxes[1] * MAX_SPEED_MULT;
    
    //Set left/right based on right joystick left/right
    msg.angular.z = lastAxes[3] * MAX_TURN_MULT;
    
    //Lets not spam 0 cmd vel messages
    if(msg.linear.x == 0 && msg.angular.z == 0)
    {
      if(firstZeroVel == true || publishAnyway == true) //Check if its the first (or close to) or if we have the e-stop override
      {
	pub.publish(msg);
	firstZeroVel = false;
      }
    }
    else
    {
      pub.publish(msg);
    }
    
    ros::spinOnce();
    
    loop_rate.sleep();
  }


  return 0;
}
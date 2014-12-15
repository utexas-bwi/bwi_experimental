
#include <signal.h> 
#include <vector>
#include <string.h>
#include <iostream>
 
   
   
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>


#include "utils/comlib/SkeletonListener.h"


//true if Ctrl-C is pressed
bool g_caught_sigint=false;

//what happens when Ctrl-C is pressed
void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};



int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "skeleton_data_receiver");
	ros::NodeHandle nh;
	
	//register ctrl-c
	signal(SIGINT, sig_handler);

	//create listener
	SkeletonListener* SL = new SkeletonListener(10000);
	
	ros::Rate r(25);

	struct KinectPacket *packet;

	// Main loop:
	while (/*!viewer.wasStopped()*/ !g_caught_sigint && ros::ok())
	{
		double secs =ros::Time::now().toSec();
		//ROS_INFO("Seconds:\t%f",secs);
		
		//collect messages
		ros::spinOnce();
		
		//listen for packets
		SL->listen();
		
		if (SL->receivedNewPacket()){
			packet = SL->getPacket();
			
			if (packet->skeletonPacket.n_skels > 0){ //we have skeleton data
				
			}
			
			if (packet->audioPacket.confidence > 0.0){ //we have audio data
				
			}
		}
		
		
		r.sleep();
		
	}
}

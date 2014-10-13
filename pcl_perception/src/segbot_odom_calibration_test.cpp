#include <signal.h> 
#include <vector>
#include <string.h>
   
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/LaserScan.h>

#include <std_srvs/Empty.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

//holds the latest scan
sensor_msgs::LaserScan S;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

bool new_scan_available_flag = false;

//publisher for velocity
ros::Publisher vel_pub;

// Mutex: //
boost::mutex scan_mutex;

void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};



void 
scan_cb (const sensor_msgs::LaserScanConstPtr& input)
{
	//ROS_INFO("Heard scan: %s",input->header.frame_id.c_str());
	
	scan_mutex.lock (); 
	
	
	S = *input;
	
	/*ROS_INFO("angle_min: %f\tangle_max: %f\tangle_increment: %f", S.angle_min, S.angle_max, S.angle_increment);
	ROS_INFO("num. readings: %i", (int)S.ranges.size());
	
	float center_dist = S.ranges[214];
	ROS_INFO("center reading %f",center_dist);*/
		//state that a new cloud is available
	new_scan_available_flag = true;
	
	scan_mutex.unlock ();
}


geometry_msgs::PoseStamped getCurrentPose(){
	  ros::NodeHandle nh;
	  tf::TransformListener listener;
	  tf::StampedTransform transform;
	  geometry_msgs::PoseStamped tempPose;
	  try{
		  listener.waitForTransform("/odom", "/base_link", ros::Time::now(), ros::Duration(2)); 
		  listener.lookupTransform("/odom","/base_link",ros::Time(0), transform);
		 // ROS_INFO("Got a transform! x = %f, y = %f",transform.getOrigin().x(),transform.getOrigin().y());
		  tempPose.pose.position.x = transform.getOrigin().x();
		  tempPose.pose.position.y = transform.getOrigin().y();
		  tempPose.pose.position.z = 0;
		  tempPose.pose.orientation.w = transform.getRotation().getW();
		  
		  tf::Vector3 angle = transform.getRotation().getAxis();
		  tempPose.pose.orientation.x = angle[0];
		  tempPose.pose.orientation.y = angle[1];
		  tempPose.pose.orientation.z = angle[2];
		  
		  
	  }
	  catch (tf::TransformException ex){
	        ROS_ERROR("Error: %s", ex.what());
	  }
	  
	  return tempPose;
  }

void wait_for_next_scan(){
	
	while (true){
		
		//collect messages
		ros::spinOnce();
		
		if (new_scan_available_flag && scan_mutex.try_lock ())    // if a new cloud is available
		{
			
			
			new_scan_available_flag = false;
			
			scan_mutex.unlock ();
			
			break;
		}
	}
	
}

bool service_callback(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
	for (int i = 0; i < 10; i ++){
	
		//get current pose
		geometry_msgs::PoseStamped start_pose = getCurrentPose();
		
		//wait for next scan and save it
		wait_for_next_scan();
		sensor_msgs::LaserScan S_start = S;
		
		float center_dist = (S_start.ranges[213]+S_start.ranges[214]+S_start.ranges[215])/3.0;
		//ROS_INFO("center reading at start: %f",center_dist);
		
		
		//move amount
		float move_amount = 0.75;
		
		//move forward
		geometry_msgs::Twist cmd;
		cmd.linear.x = move_amount;
		cmd.linear.y = 0.0;
		cmd.linear.z = 0.0;
		cmd.angular.x = 0.0;
		cmd.angular.y = 0.0;
		cmd.angular.z = 0.0;
		vel_pub.publish(cmd);
		
		//wait -- sleep for a bit
		sleep(3);
		
		wait_for_next_scan();
		sensor_msgs::LaserScan S_end = S;
		
		float center_dist_end = (S_end.ranges[213]+S_end.ranges[214]+S_end.ranges[215])/3.0;
		//ROS_INFO("center reading at end: %f",center_dist_end);
		
		float diff = center_dist - center_dist_end;
		//ROS_INFO("Difference (sensor): %f",diff);
		
		sleep(1);
		
		//end pose
		geometry_msgs::PoseStamped end_pose = getCurrentPose();
		
		//compute distance traveled
		float distance_travelled = sqrt(pow((start_pose.pose.position.x-end_pose.pose.position.x),2)+pow((start_pose.pose.position.y-end_pose.pose.position.y),2));
		
		//ROS_INFO("Difference (odom): %f",distance_travelled);
		
		//move back
		cmd.linear.x = - move_amount;
		vel_pub.publish(cmd);
		sleep(3);
		
		printf("%f,%f\n",diff,distance_travelled);
		
	}
	
	return true;
	
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segbot_odom_calibration_test");
	ros::NodeHandle nh;
	
	//register ctrl-c
	signal(SIGINT, sig_handler);

	//publisher for motor commands
	vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	
	//subscribe to scan topic
	ros::Subscriber sub_laser = nh.subscribe ("/scan", 1, scan_cb);
	
	//advertise service
	ros::ServiceServer detection_service = nh.advertiseService("segbot_odom_calibration_test/test", service_callback);

	
	while (/*!viewer.wasStopped()*/ !g_caught_sigint && ros::ok())
	{
		//collect messages
		ros::spinOnce();
	}
}

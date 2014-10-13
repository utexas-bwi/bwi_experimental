#include <signal.h> 
#include <vector>
#include <string.h>
   
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//some custom functions
#include "utils/file_io.h"
#include "utils/viz_utils.h" 
#include "utils/Grouper.h"
#include "utils/pcl_utils.h"

//the srv
#include "pcl_perception/PeopleDetectionSrv.h"

const std::string data_topic = "nav_kinect/depth_registered/points"; 

enum node_states {IDLE, DETECTING};
int node_state = IDLE; 
const int num_frames_for_detection = 5;
int current_frame_counter = 0;
bool useVoxelGridFilter = false;
bool useStatOutlierFilter = false;

//true if Ctrl-C is pressed
bool g_caught_sigint=false;

// Mutex: //
boost::mutex cloud_mutex;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud (new PointCloudT);


// viewer
bool visualize = true;
pcl::visualization::PCLVisualizer *viewer_display;   

void sig_handler(int sig)
{
  g_caught_sigint = true;
  ROS_INFO("caught sigint, init shutdown sequence...");
  ros::shutdown();
  exit(1);
};

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	//ROS_INFO("Heard cloud: %s",input->header.frame_id.c_str());
	
	cloud_mutex.lock (); 
	
	//convert to PCL format
	pcl::fromROSMsg (*input, *cloud);

	//state that a new cloud is available
	new_cloud_available_flag = true;
	
	
	
	cloud_mutex.unlock ();
}

void visualizeCloud(PointCloudT::Ptr viz_cloud)
{
	viewer_display->removeAllPointClouds();
	viewer_display->removeAllShapes();
	
	pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(viz_cloud);
	viewer_display->addPointCloud<PointT> (viz_cloud, rgb, "input_cloud");
	
	//viewer_display->spinOnce();
}



bool service_callback(pcl_perception::PeopleDetectionSrv::Request  &req,
         pcl_perception::PeopleDetectionSrv::Response &res)
{
	
	
	//the node cannot be asked to detect people while someone has asked it to detect people
	if (node_state != IDLE){
		ROS_WARN("[segbot_pcl_person_detector] Person detection service may not be called until previous call is processed");
		return true;
	}
	
	node_state = DETECTING;
	
	ROS_INFO("[general_perception.cpp] Received command %s",req.command.c_str());
	
	//command for detecting people standing up
	if (req.command == "reocrd_cloud"){

		PointCloudT::Ptr aggregated_cloud (new PointCloudT);
		bool collecting_clouds = true;
		int num_saved = 0;
		
		while (collecting_clouds){
		
			
		
			//collect to grab cloud
			if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
			{
				ROS_INFO("Adding next cloud...");

				
				*aggregated_cloud += *cloud;
				
				num_saved++;
				

				if (num_saved > num_frames_for_detection){
					collecting_clouds = false;
				}

				//unlock mutex and reset flag
				new_cloud_available_flag = false;
			}
			
			//ROS_INFO("unlocking mutex...");
			cloud_mutex.unlock ();
			
			ros::spinOnce();
		}
		
		
		if (useVoxelGridFilter){
			pcl::PCLPointCloud2::Ptr pcl_pc (new pcl::PCLPointCloud2) ;
			pcl::toPCLPointCloud2( *aggregated_cloud,*pcl_pc);
		
			//filter each step
			pcl::VoxelGrid< pcl::PCLPointCloud2  > sor;
			sor.setInputCloud (pcl_pc);
			sor.setLeafSize (0.005, 0.005, 0.005);
			sor.filter (*pcl_pc); 
					
			//covert back 
			pcl::fromPCLPointCloud2(*pcl_pc, *aggregated_cloud);
		}
		
		if (useStatOutlierFilter){
		 // Create the filtering object
		  
			 pcl::StatisticalOutlierRemoval<PointT> sor;
			 sor.setInputCloud (aggregated_cloud);
			 sor.setMeanK (50);
			 sor.setStddevMulThresh (1.0);
			 sor.filter (*aggregated_cloud);
		}
		visualizeCloud(aggregated_cloud);
		
		
		
		
		/*current_frame_counter = 0;
		bool collecting_cloud = true;
		node_state = DETECTING;
		
		while (collecting_cloud){
			
			if (new_cloud_available_flag && cloud_mutex.try_lock ())    // if a new cloud is available
			{
				ROS_INFO("Detecting on frame %i",current_frame_counter);
				
				new_cloud_available_flag = false;

				
				// Draw cloud and people bounding boxes in the viewer:
				
					
				
					
				//increment counter and see if we need to stop detecting
				current_frame_counter++;
				if (current_frame_counter > num_frames_for_detection){
					collecting_cloud = false;
				}
			}
		
			cloud_mutex.unlock ();
			
			//spin to collect next point cloud
			ros::spinOnce();
			
			if (visualize){
				visualizeCloud();
			}
		}*/
	}
		
	
	node_state = IDLE;
  
	return true;
}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "segbot_general_perception");
	ros::NodeHandle nh;
	
	// Create a ROS subscriber for the input point cloud
	ROS_INFO("[general_perception.cpp] Subscribing to point cloud topic...");
	ros::Subscriber sub = nh.subscribe (data_topic, 1, cloud_cb);
	
	//advertise service
	ros::ServiceServer detection_service = nh.advertiseService("segbot_general_perception/perception_service", service_callback);

	//register ctrl-c
	signal(SIGINT, sig_handler);
	
	// Initialize new viewer:
	if (visualize){
		viewer_display = new pcl::visualization::PCLVisualizer("People Viewer"); 
		viewer_display->setCameraPosition(0,0,-2,0,-1,0,0);
	}

	
	// For timing:
	static unsigned count = 0;
	static double last = pcl::getTime ();

	// Main loop:
	while (/*!viewer.wasStopped()*/ !g_caught_sigint && ros::ok())
	{
		//collect messages
		ros::spinOnce();
		
		if (visualize){
			viewer_display->spinOnce();
		}
					
	}

	return 0;
}





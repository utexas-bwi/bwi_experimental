
#include "ros/ros.h"
#include "bwi_scavenger/ColorShirt.h"
#include "bwi_scavenger/ObjectDetection.h"
#include "bwi_scavenger/Whiteboard.h"
#include "bwi_scavenger/VisionTask.h"

ros::NodeHandle *nh; 

bool callback_vision(bwi_scavenger::VisionTask::Request &req, 
    bwi_scavenger::VisionTask::Response &res) {
    
    ros::ServiceClient client_whiteboard = nh->serviceClient 
        <bwi_scavenger::Whiteboard> ("whiteboard_service");
    bwi_scavenger::Whiteboard srv_whiteboard;

    ros::ServiceClient client_template = nh->serviceClient
        <bwi_scavenger::ObjectDetection> ("object_detection_service");
    bwi_scavenger::ObjectDetection srv_template;

    ros::ServiceClient client_shirt = nh->serviceClient
        <bwi_scavenger::ColorShirt> ("blue_shirt_service"); 
    bwi_scavenger::ColorShirt srv_shirt;

    switch ((int)req.type) {
        
        case bwi_scavenger::VisionTaskRequest::WHITEBOARD:

            // whiteboard

            client_whiteboard.call(srv_whiteboard); 
            res.path_to_image = srv_whiteboard.response.path_to_image; 
            break;

        case bwi_scavenger::VisionTaskRequest::TEMPLATE:

            // template match

            srv_template.request.path_to_template = req.path_to_template;
            client_template.call(srv_template); 
            res.path_to_image = srv_template.response.path_to_image; 
            break;

        case bwi_scavenger::VisionTaskRequest::COLORSHIRT:

            // color shirt

            srv_shirt.request.color = req.color; 
            client_shirt.call(srv_shirt); 
            res.path_to_image = srv_shirt.response.path_to_image; 
            break;

        default:

            ROS_ERROR("Error in calling scavenger hunt vision task"); 
        
    }

    return true;
    
}

int main(int argc, char ** argv) {
    
    ros::init(argc, argv, "scavenger_vision_server"); 
    nh = new ros::NodeHandle(); 

    ros::ServiceServer service = nh->advertiseService("scavenger_vision_service", 
        callback_vision);
    ros::spin();
    
    return true;    
}


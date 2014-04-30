
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <message_filters/subscriber.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>

#include <bwi_planning_common/PlannerAtom.h>
#include <bwi_planning_common/PlannerInterface.h>
#include <bwi_planning_common_new/ActionInterfaceAction.h>
#include <segbot_logical_translator/segbot_logical_translator.h>

using bwi_planning_common::PlannerAtom;
using bwi_planning_common::NO_DOOR_IDX;

class SegbotLogicalNavigator2 : 
  public segbot_logical_translator::SegbotLogicalTranslator {

  public:

    SegbotLogicalNavigator2();
    bool sense(bwi_planning_common::PlannerInterface::Request &req,
        bwi_planning_common::PlannerInterface::Response &res);
    void execute(
        const bwi_planning_common_new::ActionInterfaceGoalConstPtr &goal);

  protected:

    void senseState(std::vector<PlannerAtom>& observations, 
        size_t door_idx = NO_DOOR_IDX);
    bool approachDoor(const std::string& door_name, 
        std::vector<PlannerAtom>& observations,
        std::string& error_message, bool gothrough = false);
    bool senseDoor(const std::string& door_name, 
        std::vector<PlannerAtom>& observations,
        std::string& error_message);
    
    bool approachObject(const std::string& object_name,
        std::vector<PlannerAtom>& observations,
        std::string& error_message);

    bool executeNavigationGoal( const geometry_msgs::PoseStamped& pose);

    void odometryHandler( const nav_msgs::Odometry::ConstPtr& odom);

    ros::NodeHandle nh1_;

    float robot_x_;
    float robot_y_;
    float robot_yaw_;

    double door_proximity_distance_;

    ros::ServiceServer service_;
    boost::shared_ptr<
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> > 
      robot_controller_;

    boost::shared_ptr<tf::TransformListener> tf_;
    boost::shared_ptr<tf::MessageFilter<nav_msgs::Odometry> > tf_filter_;
    boost::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry> > 
      odom_subscriber_;

    actionlib::SimpleActionServer<bwi_planning_common_new::ActionInterfaceAction>
      action_server_;
    bwi_planning_common_new::ActionInterfaceFeedback feedback_;
    bwi_planning_common_new::ActionInterfaceResult result_;
};

SegbotLogicalNavigator2::SegbotLogicalNavigator2() 
  : robot_x_(0), robot_y_(0), robot_yaw_(0), action_server_(
      nh1_, "execute_logical_goal", 
      boost::bind(&SegbotLogicalNavigator2::execute, this, _1), false) {

  ROS_INFO("SegbotLogicalNavigator2: Advertising sensing services!");

  ros::param::param("~door_proximity_distance", door_proximity_distance_, 1.5);

  service_ = nh_->advertiseService("sense_logical_state",
      &SegbotLogicalNavigator2::sense, this);

  robot_controller_.reset(
      new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
        "move_base", true));
  robot_controller_->waitForServer();
  
  tf_.reset(new tf::TransformListener);
  odom_subscriber_.reset(new message_filters::Subscriber<nav_msgs::Odometry>);
  odom_subscriber_->subscribe(*nh_, "odom", 5);
  tf_filter_.reset(new tf::MessageFilter<nav_msgs::Odometry>(
        *odom_subscriber_, *tf_, global_frame_id_, 5));
  tf_filter_->registerCallback(
      boost::bind(&SegbotLogicalNavigator2::odometryHandler, this, _1));

  action_server_.start();
}

void SegbotLogicalNavigator2::execute(
    const bwi_planning_common_new::ActionInterfaceGoalConstPtr& goal) {
  
  ROS_INFO("SegbotLogicalNavigator2: Advertising action services!");

  if (goal->command.name == "approach") {
    result_.success = approachDoor(goal->command.value[0], result_.observations,
        feedback_.status, false);
  } else if (goal->command.name == "gothrough") {
    result_.success = approachDoor(goal->command.value[0], result_.observations,
        feedback_.status, true);
  } else if (goal->command.name == "sensedoor") {
    result_.success = senseDoor(goal->command.value[0], result_.observations,
        feedback_.status);
  } else if (goal->command.name == "goto") {
    result_.success = approachObject(goal->command.value[0], 
        result_.observations, feedback_.status);
  } else {
    result_.success = false;
    feedback_.status = "";
  }

  if (result_.success) {
    ROS_INFO("segbot_logical_navigator2: Succeeded");
    action_server_.setSucceeded(result_);
  }
}

void SegbotLogicalNavigator2::senseState(
    std::vector<PlannerAtom>& observations, size_t door_idx) {

  PlannerAtom at;
  at.name = "at";
  size_t location_idx = getLocationIdx(bwi::Point2f(robot_x_, robot_y_));
  at.value.push_back(getLocationString(location_idx));
  observations.push_back(at);

  size_t num_doors = getNumDoors();
  bwi::Point2f robot_loc(robot_x_, robot_y_);
  bool first_facing = false;
  bool first_beside = false;
  size_t facing_idx = NO_DOOR_IDX;

  for (size_t door = 0; door < num_doors; ++door) {
    if (door_idx != NO_DOOR_IDX && door != door_idx) {
      continue;
    }
    bool facing_door = !first_facing && 
      isRobotFacingDoor(robot_loc, robot_yaw_, door_proximity_distance_, door);
    bool beside_door = !first_beside && 
      isRobotBesideDoor(robot_loc, robot_yaw_, door_proximity_distance_, door);
    if (!facing_door) {
      PlannerAtom n_facing;
      n_facing.name = "-facing";
      n_facing.value.push_back(getDoorString(door));
      observations.push_back(n_facing);
    } else {
      PlannerAtom facing;
      facing.name = "facing";
      facing.value.push_back(getDoorString(door));
      observations.push_back(facing);
      first_facing = true;
      facing_idx = door_idx;
    }
    if (!beside_door) {
      PlannerAtom n_beside;
      n_beside.name = "-beside";
      n_beside.value.push_back(getDoorString(door));
      observations.push_back(n_beside);
    } else {
      PlannerAtom beside;
      beside.name = "beside";
      beside.value.push_back(getDoorString(door));
      observations.push_back(beside);
      first_beside = true;
    }
  }

  // If we are facing a door, also sense whether it is open
  if (facing_idx != NO_DOOR_IDX) {
    PlannerAtom door_open;
    door_open.value.push_back(getDoorString(facing_idx));
    if (isDoorOpen(facing_idx)) {
      door_open.name = "open";
      observations.push_back(door_open);
    } else {
      door_open.name = "-open";
      observations.push_back(door_open);
    }
  }
}

bool SegbotLogicalNavigator2::executeNavigationGoal(
    const geometry_msgs::PoseStamped& pose) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = pose;
  robot_controller_->sendGoal(goal);
  robot_controller_->waitForResult();
  actionlib::SimpleClientGoalState state = robot_controller_->getState();
  return state == actionlib::SimpleClientGoalState::SUCCEEDED;
}

void SegbotLogicalNavigator2::odometryHandler(
    const nav_msgs::Odometry::ConstPtr& odom) {
  geometry_msgs::PoseStamped pose_in, pose_out;
  pose_in.header = odom->header;
  pose_in.pose = odom->pose.pose;
  tf_->transformPose(global_frame_id_, pose_in, pose_out);
  robot_x_ = pose_out.pose.position.x;
  robot_y_ = pose_out.pose.position.y;
  robot_yaw_ = tf::getYaw(pose_out.pose.orientation);
}

bool SegbotLogicalNavigator2::approachDoor(const std::string& door_name, 
    std::vector<PlannerAtom>& observations,
    std::string& error_message, bool gothrough) {
  error_message = "";

  size_t door_idx = getDoorIdx(door_name);
  if (door_idx == NO_DOOR_IDX) {
    // Interface failure
    error_message = "Could not resolve argument: " + door_name;
    return false;
  } else {

    bwi::Point2f approach_pt;
    float approach_yaw = 0;
    bool door_approachable = false;

    if (!gothrough) {
      door_approachable = getApproachPoint(door_idx, 
          bwi::Point2f(robot_x_, robot_y_), approach_pt, approach_yaw);
    } else {
      door_approachable = getThroughDoorPoint(door_idx,
          bwi::Point2f(robot_x_, robot_y_), approach_pt, approach_yaw);
    }
    
    if (door_approachable) {
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = global_frame_id_;
      pose.pose.position.x = approach_pt.x;
      pose.pose.position.y = approach_pt.y;
      tf::quaternionTFToMsg(
          tf::createQuaternionFromYaw(approach_yaw), pose.pose.orientation); 
      bool success = executeNavigationGoal(pose);

      // Publish the observable fluents
      senseState(observations, door_idx);

      return success;
    } else {
      // Planning failure
      error_message = "Cannot interact with " + door_name + " from here.";
      return false;
    }
  }
}

bool SegbotLogicalNavigator2::approachObject(const std::string& object_name, 
    std::vector<PlannerAtom>& observations,
    std::string& error_message) {

  error_message = "";
  observations.clear();

  if (object_approach_map_.find(object_name) != object_approach_map_.end()) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = global_frame_id_;
    pose.pose = object_approach_map_[object_name];
    bool success = executeNavigationGoal(pose);

    // Publish the observable fluents
    senseState(observations, NO_DOOR_IDX);
    PlannerAtom closeto;
    closeto.name = "closeto";
    closeto.value.push_back(object_name);
    if (!success) {
      closeto.name = "-closeto";
    }
    observations.push_back(closeto);
  }

  error_message = object_name + " does not exist.";
  return false;
}

bool SegbotLogicalNavigator2::senseDoor(const std::string& door_name, 
    std::vector<PlannerAtom>& observations,
    std::string& error_message) {
  error_message = "";
  size_t door_idx = 
    bwi_planning_common::resolveDoor(door_name, doors_);
  if (door_idx == bwi_planning_common::NO_DOOR_IDX) {
    error_message = "Door " + door_name + " does not exist!";
    return false;
  } 
  bool door_open = isDoorOpen(door_idx);
  PlannerAtom open;
  open.name = "open";
  if (!door_open) {
    open.name = "-" + open.name;
  }
  open.value.push_back(door_name);
  observations.push_back(open);
  return true;
}

bool SegbotLogicalNavigator2::sense(
    bwi_planning_common::PlannerInterface::Request &req,
    bwi_planning_common::PlannerInterface::Response &res) {

  res.observations.clear();

//  if (req.command.name == "approach") {
//    res.success = approachDoor(req.command.value[0], res.observations,
//        res.status, false);
//  } else if (req.command.name == "gothrough") {
//    res.success = approachDoor(req.command.value[0], res.observations,
//        res.status, true);
//  } else if (req.command.name == "sensedoor") {
//    res.success = senseDoor(req.command.value[0], res.observations,
//        res.status);
//  } else if (req.command.name == "goto") {
//    res.success = approachObject(req.command.value[0], res.observations,
//        res.status);
//  } else {
//    res.success = true;
//    res.status = "";
//    senseState(res.observations);
//  }

  res.success = true;
  res.status = "";
  senseState(res.observations);

  return true;
}


int main(int argc, char *argv[]) {
  
  ros::init(argc, argv, "segbot_logical_navigator2");
  ros::NodeHandle nh;

  ROS_INFO("SegbotLogicalNavigator2: Starting up node...");
  SegbotLogicalNavigator2 handler;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  ROS_INFO("SegbotLogicalNavigator2: Stopping node.");

  return 0;
}

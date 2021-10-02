// Navigation Related Behaviors

#include "navigation_behaviors.h"
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "yaml-cpp/yaml.h"


// SETWAYPOINTS
// Gets a list of waypoints from a YAML file and ensures they are not empty
SetWaypoints::SetWaypoints(const std::string& name, const BT::NodeConfiguration& config) :
    BT::SyncActionNode(name, config)
{
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

BT::NodeStatus SetWaypoints::tick()
{
    std::string yaml_file;
    ros::param::get("waypoint_list", yaml_file);
    YAML::Node waypoints = YAML::LoadFile(yaml_file);
    int num_wp = waypoints.size();
    if (num_wp == 0) {
        std::cout << "[" << this->name() << "] No waypoints found." << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    setOutput("num_wp", num_wp);
    std::cout << "[" << this->name() << "] Found " << num_wp << " waypoints." << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::PortsList SetWaypoints::providedPorts()
{
    return { BT::OutputPort<int>("num_wp") };
}


// GETWAYPOINTFROMQUEUE
// Gets a waypoint name from a queue of waypoints to visit.
// If the queue is empty, this behavior fails.
GetWaypointFromQueue::GetWaypointFromQueue(const std::string& name, const BT::NodeConfiguration& config) :
    BT::SyncActionNode(name, config)
{
    std::string yaml_file;
    ros::param::get("waypoint_list", yaml_file);
    YAML::Node waypoints = YAML::LoadFile(yaml_file);
    for(YAML::const_iterator it=waypoints.begin(); it!=waypoints.end(); ++it) {
        waypoint_queue_.push_back(it->first.as<std::string>());
    }
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

BT::NodeStatus GetWaypointFromQueue::tick()
{
    if (waypoint_queue_.empty()) {
        std::cout << "[" << this->name() << "] No more waypoints!" << std::endl;
        std::cout << "[" << this->name() << "] Starting sequence again..." << std::endl;
        std::string yaml_file;
        ros::param::get("waypoint_list", yaml_file);
        YAML::Node waypoints = YAML::LoadFile(yaml_file);
        for(YAML::const_iterator it=waypoints.begin(); it!=waypoints.end(); ++it) {
            waypoint_queue_.push_back(it->first.as<std::string>());
        }
        if(waypoint_queue_.empty()){
           std::cout << "[" << this->name() << "] No waypoints available!" << std::endl;
           return BT::NodeStatus::FAILURE; 
        } 
    }
    
    if(!waypoint_queue_.empty()){
        std::string tgt_wp = waypoint_queue_.front();
        setOutput("target_wp", tgt_wp);
        waypoint_queue_.pop_front();
        std::cout << "[" << this->name() << "] Getting next goal: " << tgt_wp << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
}

BT::PortsList GetWaypointFromQueue::providedPorts()
{
    return { BT::OutputPort<std::string>("target_wp") };
}


// GOTOGOAL
// Wrapper behavior around a `move_base` action client, whose status
// reflects the status of the ROS action.
GoToGoal::GoToGoal(const std::string& name, const BT::NodeConfiguration& config) :
    BT::StatefulActionNode(name, config), 
    client_("move_base", true)
{
    client_.waitForServer();
    std::cout << "[" << this->name() << "] Initialized" << std::endl;
}

BT::NodeStatus GoToGoal::onStart() {    
    BT::Optional<std::string> goal = getInput<std::string>("goal");
    std::string yaml_file;
    ros::param::get("waypoint_list", yaml_file);
    YAML::Node waypoints = YAML::LoadFile(yaml_file);
    std::vector<float> pose = waypoints[goal.value()].as<std::vector<float>>();

    std::cout << "[" << this->name() << "] Sending goal..." << std::endl;
    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.pose.position.x = pose[0];
    goal_.target_pose.pose.position.y = pose[1];
    tf2::Quaternion q;
    q.setRPY(0, 0, pose[2]);
    q.normalize();
    goal_.target_pose.pose.orientation = tf2::toMsg(q);
    client_.sendGoal(goal_);
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToGoal::onRunning() {
    actionlib::SimpleClientGoalState state = client_.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        std::cout << "[" << this->name() << "] Goal reached" << std::endl;
        return BT::NodeStatus::SUCCESS;
    } else if (state == actionlib::SimpleClientGoalState::ACTIVE) {
        return BT::NodeStatus::RUNNING;
    } else {
        std::cout << "[" << this->name() << "] Failed to reach goal" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

void GoToGoal::onHalted() {};

BT::PortsList GoToGoal::providedPorts() {
    return { BT::InputPort<std::string>("goal") };
}

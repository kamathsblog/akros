// Main node for waypoint_sequence behavior

#include "ros/ros.h"
#include "yaml-cpp/yaml.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
// Todo: fix realtime monitoring groot
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h" 
#include "navigation_behaviors.h"

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "waypoint_sequence");
    ros::NodeHandle nh;

    // Read YAML file
    std::string yaml_file;
    ros::param::get("waypoint_list", yaml_file);
    YAML::Node waypoints = YAML::LoadFile(yaml_file);
    std::vector<std::string> wp_names;
    for(YAML::const_iterator it=waypoints.begin(); it!=waypoints.end(); ++it) {
        wp_names.push_back(it->first.as<std::string>());
    }

    // Build a behavior tree from XML and (todo: set it up for logging using groot)
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<GoToGoal>("GoToGoal");
    factory.registerNodeType<SetWaypoints>("SetWaypoints");
    factory.registerNodeType<GetWaypointFromQueue>("GetWaypointFromQueue");
    auto tree = factory.createTreeFromFile(BT_XML_PATH);

    // Todo: fix realtime monitoring groot
    //BT::PublisherZMQ publisher_zmq(tree);

    // Tick the tree until it reaches a terminal state
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING) {
        status = tree.tickRoot();
        ros::Duration(0.5).sleep();
    }

    // Output final results
    std::string status_str;
    if (status == BT::NodeStatus::SUCCESS) {
        status_str = "SUCCESS";
    } else {
        status_str = "FAILURE";
    }
    ROS_INFO("Done with status %s!", status_str.c_str());
    return 0;
}

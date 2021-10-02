// Navigation behaviors

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Sets number of waypoints from list
class SetWaypoints : public BT::SyncActionNode
{
  public:
    SetWaypoints(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

// Gets waypoints from a queue of locations read from a list
class GetWaypointFromQueue : public BT::SyncActionNode
{
  public:

    std::deque<std::string> waypoint_queue_;

    GetWaypointFromQueue(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus tick() override;
    static BT::PortsList providedPorts();
};

// Go to a target location (wraps around `move_base`)
class GoToGoal : public BT::StatefulActionNode
{
  public:

    MoveBaseClient client_;
    move_base_msgs::MoveBaseGoal goal_;

    GoToGoal(const std::string& name, const BT::NodeConfiguration& config);
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
    static BT::PortsList providedPorts();
};
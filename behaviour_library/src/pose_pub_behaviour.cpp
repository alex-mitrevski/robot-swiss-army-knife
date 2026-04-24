#include "behaviour_library/pose_pub_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

PosePubBehaviour::PosePubBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{}

PortsList PosePubBehaviour::providedPorts()
{
    return providedBasicPorts({
        InputPort<geometry_msgs::msg::PoseStamped>("object_pose")
    });
}

bool PosePubBehaviour::setMessage(geometry_msgs::msg::PoseStamped& pose_stamped_msg)
{
    this->getInput("object_pose", pose_stamped_msg);
    return true;
}

CreateRosNodePlugin(PosePubBehaviour, "PosePubBehaviour");
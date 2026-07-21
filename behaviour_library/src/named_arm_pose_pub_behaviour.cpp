#include "behaviour_library/named_arm_pose_pub_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

NamedArmPosePubBehaviour::NamedArmPosePubBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosTopicPubNode<std_msgs::msg::String>(name, conf, params)
{}

PortsList NamedArmPosePubBehaviour::providedPorts()
{
    return providedBasicPorts({
        InputPort<std::string>("pose_name")
    });
}

bool NamedArmPosePubBehaviour::setMessage(std_msgs::msg::String& string_msg)
{
    this->getInput("pose_name", string_msg.data);
    return true;
}

CreateRosNodePlugin(NamedArmPosePubBehaviour, "NamedArmPosePubBehaviour");
#include "behaviour_library/bool_sub_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

BoolSubBehaviour::BoolSubBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosTopicSubNode<std_msgs::msg::Bool>(name, conf, params)
{}

PortsList BoolSubBehaviour::providedPorts()
{
    return providedBasicPorts({});
}

NodeStatus BoolSubBehaviour::onTick(const std::shared_ptr<std_msgs::msg::Bool>& msg)
{
    if(msg && msg->data)
    {
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

CreateRosNodePlugin(BoolSubBehaviour, "BoolSubBehaviour");
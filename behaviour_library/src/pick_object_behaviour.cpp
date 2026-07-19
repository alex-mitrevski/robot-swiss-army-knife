#include "behaviour_library/pick_object_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

PickObjectBehaviour::PickObjectBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosActionNode<pick_skill::action::PickSkill>(name, conf, params)
{}

PortsList PickObjectBehaviour::providedPorts()
{
    return providedBasicPorts({
        InputPort<std::vector<robot_swiss_knife_msgs::msg::Object>>("object_to_grasp")
    });
}


bool PickObjectBehaviour::setGoal(RosActionNode::Goal& goal)
{
    this->getInput("object_to_grasp", goal.object);
    return true;
}

NodeStatus PickObjectBehaviour::onResultReceived(const WrappedResult& wrapped_result)
{
    if (wrapped_result.result->success)
    {
        RCLCPP_INFO(this->logger(), "Pick object action successful");
        if (wrapped_result.result->value.length() > 0)
        {
            RCLCPP_INFO(this->logger(), "Execution feedback: %s", wrapped_result.result->value.c_str());
        }
        return NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(this->logger(), "Pick object action unsuccessful: %s", wrapped_result.result->value.c_str());
    return NodeStatus::FAILURE;
}

NodeStatus PickObjectBehaviour::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(this->logger(), "Pick object action error: %d", error);
    return NodeStatus::FAILURE;
}

NodeStatus PickObjectBehaviour::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    if (feedback->feedback.length() > 0)
    {
        RCLCPP_INFO(this->logger(), "Pick object action feedback: %s", feedback->feedback.c_str());
    }
    return NodeStatus::RUNNING;
}

CreateRosNodePlugin(PickObjectBehaviour, "PickObjectBehaviour");
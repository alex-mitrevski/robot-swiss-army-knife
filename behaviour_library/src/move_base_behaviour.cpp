#include "behaviour_library/move_base_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

MoveBaseBehaviour::MoveBaseBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosActionNode<move_base_skill::action::MoveBaseSkill>(name, conf, params)
{}

PortsList MoveBaseBehaviour::providedPorts()
{
    return providedBasicPorts({
        InputPort<int>("goal_type"),
        InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goal_poses")
    });
}


bool MoveBaseBehaviour::setGoal(RosActionNode::Goal& goal)
{
    this->getInput("goal_type", goal.goal_type);
    if (goal.goal_type == move_base_skill::action::MoveBaseSkill::Goal::POSE)
    {
        this->getInput("goal_poses", goal.poses);
    }
    else
    {
        RCLCPP_ERROR(this->logger(), "Unknown or unsupported goal type %d for move base action; ignoring request", goal.goal_type);
        return false;
    }
    return true;
}

NodeStatus MoveBaseBehaviour::onResultReceived(const WrappedResult& wrapped_result)
{
    if (wrapped_result.result->success)
    {
        RCLCPP_INFO(this->logger(), "Move base action successful");
        if (wrapped_result.result->value.length() > 0)
        {
            RCLCPP_INFO(this->logger(), "Execution feedback: %s", wrapped_result.result->value.c_str());
        }
        return NodeStatus::SUCCESS;
    }
    RCLCPP_ERROR(this->logger(), "Move base action unsuccessful: %s", wrapped_result.result->value.c_str());
    return NodeStatus::FAILURE;
}

NodeStatus MoveBaseBehaviour::onFailure(ActionNodeErrorCode error)
{
    RCLCPP_ERROR(this->logger(), "Move base action error: %d", error);
    return NodeStatus::FAILURE;
}

NodeStatus MoveBaseBehaviour::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
    if (feedback->feedback.length() > 0)
    {
        RCLCPP_INFO(this->logger(), "Move base action feedback: %s", feedback->feedback.c_str());
    }
    return NodeStatus::RUNNING;
}

CreateRosNodePlugin(MoveBaseBehaviour, "MoveBaseBehaviour");
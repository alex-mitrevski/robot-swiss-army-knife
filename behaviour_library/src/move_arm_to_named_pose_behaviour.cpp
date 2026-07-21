#include "behaviour_library/move_arm_to_named_pose_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

MoveArmToNamedPoseBehaviour::MoveArmToNamedPoseBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: StatefulActionNode(name, conf), node(params.nh)
{
    this->getInput("goal_topic_name", this->goal_topic_name);
    this->getInput("result_topic_name", this->result_topic_name);
    this->getInput("pose_name", this->pose_name);
}

PortsList MoveArmToNamedPoseBehaviour::providedPorts()
{
    return { InputPort<std::string>("goal_topic_name"),
             InputPort<std::string>("result_topic_name"),
             InputPort<std::string>("pose_name") };
}

NodeStatus MoveArmToNamedPoseBehaviour::onStart()
{
    this->execution_result_msg.reset();
    this->result_sub = this->node->create_subscription<std_msgs::msg::Bool>(this->result_topic_name,
                                                                            rclcpp::SystemDefaultsQoS(),
                                                                            std::bind(&MoveArmToNamedPoseBehaviour::result_cb, this, _1));

    this->goal_pub = this->node->create_publisher<std_msgs::msg::String>(this->goal_topic_name, 10);
    std_msgs::msg::String goal_msg;
    goal_msg.data = this->pose_name;
    this->goal_pub->publish(goal_msg);

    return NodeStatus::RUNNING;
}

NodeStatus MoveArmToNamedPoseBehaviour::onRunning()
{
    if (this->execution_result_msg)
    {
        this->result_sub.reset();
        if (this->execution_result_msg->data)
        {
            return NodeStatus::SUCCESS;
        }
        return NodeStatus::FAILURE;
    }
    return NodeStatus::RUNNING;
}

void MoveArmToNamedPoseBehaviour::onHalted()
{
    this->result_sub.reset();
}

void MoveArmToNamedPoseBehaviour::result_cb(std_msgs::msg::Bool::SharedPtr msg)
{
    this->execution_result_msg = msg;
}

CreateRosNodePlugin(MoveArmToNamedPoseBehaviour, "MoveArmToNamedPoseBehaviour");
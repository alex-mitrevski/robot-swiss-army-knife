#ifndef MOVE_ARM_TO_NAMED_POSE_BEHAVIOUR_HPP
#define MOVE_ARM_TO_NAMED_POSE_BEHAVIOUR_HPP

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::placeholders;
using namespace BT;

/**
 * A behaviour that publishes a String message from with the name of a predefined
 * pose to which a robot's arm should move and waits for a result to be received.
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class MoveArmToNamedPoseBehaviour : public StatefulActionNode
{
public:
    MoveArmToNamedPoseBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);
    static PortsList providedPorts();
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;
    void result_cb(std_msgs::msg::Bool::SharedPtr msg);
private:
    rclcpp::Node::SharedPtr node;
    std::string goal_topic_name;
    std::string result_topic_name;
    std::string pose_name;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr result_sub;
    std_msgs::msg::Bool::SharedPtr execution_result_msg;
};

#endif
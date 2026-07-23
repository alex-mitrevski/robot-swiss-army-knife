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

    /**
     * Sets the ports of the behaviour:
     * - Input ports:
     *     * goal_topic_name (std::string)
     *     * result_topic_name (std::string)
     *     * pose_name (std::string)
     */
    static PortsList providedPorts();

    /**
     * Sends an arm motion request and initialises a subscriber
     * that waits for feedback. Clears any prior messages
     * that have been received.
     */
    NodeStatus onStart() override;

    /**
     * If an arm execution result message has been received, returns
     * SUCCESS or FAILURE depending on the message contents.
     * Returns RUNNING otherwise.
     */
    NodeStatus onRunning() override;

    /**
     * Cleans up the component by stopping the execution result subscriber.
     */
    void onHalted() override;

    /**
     * Stores the received message.
     */
    void result_cb(std_msgs::msg::Bool::SharedPtr msg);
private:
    /**
     * Topic on which named arm goals can be sent.
     */
    std::string goal_topic_name;

    /**
     * Topic on which arm execution results are received.
     */
    std::string result_topic_name;

    /**
     * Name of an arm pose to which a robot should be sent.
     */
    std::string pose_name;

    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr result_sub;
    std_msgs::msg::Bool::SharedPtr execution_result_msg;
};

#endif
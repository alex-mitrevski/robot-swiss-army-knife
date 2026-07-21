#ifndef BOOL_SUB_BEHAVIOUR_HPP
#define BOOL_SUB_BEHAVIOUR_HPP

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace BT;

/**
 * A behaviour that subscribes to a Bool message and returns success or
 * failure based on the message value. Based on the samples in
 * https://github.com/BehaviorTree/BehaviorTree.ROS2
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class BoolSubBehaviour : public RosTopicSubNode<std_msgs::msg::Bool>
{
public:
    BoolSubBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    /**
     * The behaviour provides no input or output ports.
     */
    static PortsList providedPorts();

    /**
     * Returns SUCCESS or FAILURE depending on the value of the received message.
     */
    virtual NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Bool>& msg) override;
};

#endif
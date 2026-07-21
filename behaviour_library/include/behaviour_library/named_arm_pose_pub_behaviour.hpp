#ifndef NAMED_ARM_POSE_PUB_BEHAVIOUR_HPP
#define NAMED_ARM_POSE_PUB_BEHAVIOUR_HPP

#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <std_msgs/msg/string.hpp>

using namespace BT;

/**
 * A behaviour that publishes a String message from with the name of a predefined
 * pose to which a robot's arm should move. Based on the samples in
 * https://github.com/BehaviorTree/BehaviorTree.ROS2
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class NamedArmPosePubBehaviour : public RosTopicPubNode<std_msgs::msg::String>
{
public:
    NamedArmPosePubBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    /**
     * Sets the ports of the behaviour:
     * - Input ports:
     *     * pose_name (string)
     */
    static PortsList providedPorts();

    /**
     * Sets the message based on the blackboard contents.
     */
    virtual bool setMessage(std_msgs::msg::String& string_msg) override;
};

#endif
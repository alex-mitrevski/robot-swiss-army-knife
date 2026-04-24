#ifndef POSE_PUB_BEHAVIOUR_HPP
#define POSE_PUB_BEHAVIOUR_HPP

#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace BT;

/**
 * A behaviour that publishes a PoseStamped message that is saved on the blackboard.
 * Based on the samples in https://github.com/BehaviorTree/BehaviorTree.ROS2
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class PosePubBehaviour : public RosTopicPubNode<geometry_msgs::msg::PoseStamped>
{
public:
    PosePubBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    /**
     * Sets the ports of the behaviour:
     * - Input ports:
     *     * object_pose (geometry_msgs::msg::PoseStamped)
     */
    static PortsList providedPorts();

    /**
     * Sets the message based on the blackboard contents.
     */
    virtual bool setMessage(geometry_msgs::msg::PoseStamped& pose_stamped_msg) override;
};

#endif
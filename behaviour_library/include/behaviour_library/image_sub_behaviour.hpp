#ifndef IMAGE_SUB_BEHAVIOUR_HPP
#define IMAGE_SUB_BEHAVIOUR_HPP

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace BT;

/**
 * A behaviour that subscribes to an image and writes the result to the blackboard.
 * Based on the samples in https://github.com/BehaviorTree/BehaviorTree.ROS2
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class ImageSubBehaviour : public RosTopicSubNode<sensor_msgs::msg::Image>
{
public:
    ImageSubBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    /**
     * Sets the ports of the behaviour:
     * - Output ports:
     *     * latest_image (sensor_msgs::msg::Image)
     */
    static PortsList providedPorts();

    /**
     * Updates the blackboard with the received message and returns SUCCESS.
     * If no message has been received, returns FAILURE.
     */
    virtual NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::Image>& last_msg) override;
};

#endif
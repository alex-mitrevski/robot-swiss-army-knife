#ifndef CLOUD_SUB_BEHAVIOUR_HPP
#define CLOUD_SUB_BEHAVIOUR_HPP

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace BT;

/**
 * A behaviour that subscribes to a point cloud and writes the result to the blackboard.
 * Based on the samples in https://github.com/BehaviorTree/BehaviorTree.ROS2
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class PointCloudSubBehaviour : public RosTopicSubNode<sensor_msgs::msg::PointCloud2>
{
public:
    PointCloudSubBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    /**
     * Sets the ports of the behaviour:
     * - Output ports:
     *     * latest_cloud (sensor_msgs::msg::PointCloud2)
     */
    static PortsList providedPorts();

    /**
     * Updates the blackboard with the received message and returns SUCCESS.
     * If no message has been received, returns FAILURE.
     */
    virtual NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::PointCloud2>& last_msg) override;
};

#endif
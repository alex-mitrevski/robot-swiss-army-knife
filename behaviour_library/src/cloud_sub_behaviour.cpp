#include "behaviour_library/cloud_sub_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

PointCloudSubBehaviour::PointCloudSubBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosTopicSubNode<sensor_msgs::msg::PointCloud2>(name, conf, params, rclcpp::SensorDataQoS())
{}

PortsList PointCloudSubBehaviour::providedPorts()
{
    return providedBasicPorts({
        OutputPort<sensor_msgs::msg::PointCloud2>("latest_point_cloud")
    });
}

NodeStatus PointCloudSubBehaviour::onTick(const std::shared_ptr<sensor_msgs::msg::PointCloud2>& last_msg)
{
    if(last_msg)
    {
        this->setOutput("latest_point_cloud", *last_msg);
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

CreateRosNodePlugin(PointCloudSubBehaviour, "PointCloudSubBehaviour");
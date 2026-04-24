#include "behaviour_library/image_sub_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

ImageSubBehaviour::ImageSubBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosTopicSubNode<sensor_msgs::msg::Image>(name, conf, params, rclcpp::SensorDataQoS())
{}

PortsList ImageSubBehaviour::providedPorts()
{
    return providedBasicPorts({
        OutputPort<sensor_msgs::msg::Image>("latest_image")
    });
}

NodeStatus ImageSubBehaviour::onTick(const std::shared_ptr<sensor_msgs::msg::Image>& last_msg)
{
    if(last_msg)
    {
        this->setOutput("latest_image", *last_msg);
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

CreateRosNodePlugin(ImageSubBehaviour, "ImageSubBehaviour");
#include "behaviour_library/segment_objects_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

SegmentObjectsBehaviour::SegmentObjectsBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosServiceNode<robot_swiss_knife_msgs::srv::SegmentObjects>(name, conf, setCustomParams(params))
{}

PortsList SegmentObjectsBehaviour::providedPorts()
{
    return providedBasicPorts({
        InputPort<sensor_msgs::msg::Image>("latest_image"),
        InputPort<std::vector<std::string>>("object_categories"),
        OutputPort<std::vector<robot_swiss_knife_msgs::msg::Object>>("segmented_objects")
    });
}

bool SegmentObjectsBehaviour::setRequest(Request::SharedPtr& request)
{
    this->getInput("latest_image", request->image);
    this->getInput("object_categories", request->object_categories);
    return true;
}

NodeStatus SegmentObjectsBehaviour::onResponseReceived(const Response::SharedPtr& response)
{
    if (response->objects.size() == 0)
    {
        return NodeStatus::FAILURE;
    }

    this->setOutput("segmented_objects", response->objects);
    return NodeStatus::SUCCESS;
}

NodeStatus SegmentObjectsBehaviour::onFailure(ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(this->logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

RosNodeParams SegmentObjectsBehaviour::setCustomParams(RosNodeParams params)
{
    params.server_timeout = std::chrono::milliseconds(10000);
    return params;
}

CreateRosNodePlugin(SegmentObjectsBehaviour, "SegmentObjectsBehaviour");
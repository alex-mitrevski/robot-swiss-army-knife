#include "behaviour_library/check_visibility_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

CheckVisibilityBehaviour::CheckVisibilityBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosServiceNode<robot_swiss_knife_msgs::srv::CheckObjectVisibility>(name, conf, params)
{}

PortsList CheckVisibilityBehaviour::providedPorts()
{
    return providedBasicPorts({
        InputPort<sensor_msgs::msg::Image>("latest_image"),
        InputPort<std::vector<std::string>>("object_categories"),
        OutputPort<std::vector<std::string>>("visible_objects")
    });
}

bool CheckVisibilityBehaviour::setRequest(Request::SharedPtr& request)
{
    this->getInput("latest_image", request->image);
    this->getInput("object_categories", request->object_categories);
    return true;
}

NodeStatus CheckVisibilityBehaviour::onResponseReceived(const Response::SharedPtr& response)
{
    this->setOutput("visible_objects", response->objects_visible);
    return NodeStatus::SUCCESS;
}

NodeStatus CheckVisibilityBehaviour::onFailure(ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(this->logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

CreateRosNodePlugin(CheckVisibilityBehaviour, "CheckVisibilityBehaviour");
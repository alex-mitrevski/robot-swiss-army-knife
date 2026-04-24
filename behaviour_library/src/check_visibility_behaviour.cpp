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
    std::vector<std::string> object_categories;
    this->getInput("object_categories", object_categories);

    std::vector<std::string> visible_objects;
    for (unsigned int i=0; i<object_categories.size(); i++)
    {
        if (response->objects_visible[i])
        {
            visible_objects.push_back(object_categories[i]);
        }
    }
    this->setOutput("visible_objects", visible_objects);

    if (visible_objects.size() > 0)
    {
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

NodeStatus CheckVisibilityBehaviour::onFailure(ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(this->logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

RosNodeParams CheckVisibilityBehaviour::setCustomParams(RosNodeParams params)
{
    params.server_timeout = std::chrono::milliseconds(10000);
    return params;
}

CreateRosNodePlugin(CheckVisibilityBehaviour, "CheckVisibilityBehaviour");
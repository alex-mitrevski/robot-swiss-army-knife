#include "behaviour_library/extract_roi_3d_points_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

ExtractROI3DPointsBehaviour::ExtractROI3DPointsBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosServiceNode<robot_swiss_knife_msgs::srv::ExtractROI3DPoints>(name, conf, params)
{}

PortsList ExtractROI3DPointsBehaviour::providedPorts()
{
    return providedBasicPorts({
        InputPort<sensor_msgs::msg::PointCloud2>("latest_point_cloud"),
        InputPort<std::vector<robot_swiss_knife_msgs::msg::Object>>("segmented_objects"),
        OutputPort<std::vector<robot_swiss_knife_msgs::msg::Object>>("objects")
    });
}

bool ExtractROI3DPointsBehaviour::setRequest(Request::SharedPtr& request)
{
    this->getInput("latest_point_cloud", request->point_cloud);
    this->getInput("segmented_objects", request->objects);
    return true;
}

NodeStatus ExtractROI3DPointsBehaviour::onResponseReceived(const Response::SharedPtr& response)
{
    this->setOutput("objects", response->objects);
    return NodeStatus::SUCCESS;
}

NodeStatus ExtractROI3DPointsBehaviour::onFailure(ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(this->logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

RosNodeParams ExtractROI3DPointsBehaviour::setCustomParams(RosNodeParams params)
{
    params.server_timeout = std::chrono::milliseconds(10000);
    return params;
}

CreateRosNodePlugin(ExtractROI3DPointsBehaviour, "ExtractROI3DPointsBehaviour");
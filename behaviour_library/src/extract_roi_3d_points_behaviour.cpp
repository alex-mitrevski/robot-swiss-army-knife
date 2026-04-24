#include "behaviour_library/extract_roi_3d_points_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

ExtractROI3DPointsBehaviour::ExtractROI3DPointsBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosServiceNode<robot_swiss_knife_msgs::srv::ExtractROI3DPoints>(name, conf, params)
{}

PortsList ExtractROI3DPointsBehaviour::providedPorts()
{
    return providedBasicPorts({
        InputPort<sensor_msgs::msg::PointCloud2>("latest_point_cloud"),
        InputPort<std::map<std::string, sensor_msgs::msg::Image>>("segmented_objects"),
        OutputPort<std::map<std::string, sensor_msgs::msg::PointCloud2>>("object_clouds")
    });
}

bool ExtractROI3DPointsBehaviour::setRequest(Request::SharedPtr& request)
{
    this->getInput("latest_point_cloud", request->point_cloud);

    std::map<std::string, sensor_msgs::msg::Image> segmented_objects;
    this->getInput("segmented_objects", segmented_objects);

    this->object_names.clear();
    for (const auto &segmented_object_data : segmented_objects)
    {
        this->object_names.push_back(segmented_object_data.first);
        request->object_masks.push_back(segmented_object_data.second);
    }

    return true;
}

NodeStatus ExtractROI3DPointsBehaviour::onResponseReceived(const Response::SharedPtr& response)
{
    std::map<std::string, sensor_msgs::msg::PointCloud2> roi_output;
    for (unsigned int i=0; i<response->object_clouds.size(); i++)
    {
        roi_output[this->object_names[i]] = response->object_clouds[i];
    }
    this->setOutput("object_clouds", roi_output);
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
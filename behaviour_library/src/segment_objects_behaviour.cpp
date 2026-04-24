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
        OutputPort<std::map<std::string, sensor_msgs::msg::Image>>("segmented_objects")
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
    if (response->mask_categories.size() == 0)
    {
        return NodeStatus::FAILURE;
    }

    std::map<std::string, sensor_msgs::msg::Image> segmentation_output;
    std::map<std::string, int> category_obj_counters;
    for (unsigned int i=0; i<response->mask_categories.size(); i++)
    {
        if (category_obj_counters.find(response->mask_categories[i]) == category_obj_counters.end())
        {
            category_obj_counters[response->mask_categories[i]] = 0;
        }
        else
        {
            category_obj_counters[response->mask_categories[i]] += 1;
        }
        std::string obj_name = response->mask_categories[i] + "_" + std::to_string(category_obj_counters[response->mask_categories[i]]);
        segmentation_output[obj_name] = response->masks[i];
    }
    this->setOutput("segmented_objects", segmentation_output);
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
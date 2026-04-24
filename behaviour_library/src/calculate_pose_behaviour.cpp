#include "behaviour_library/calculate_pose_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

CalculatePoseBehaviour::CalculatePoseBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosServiceNode<robot_swiss_knife_msgs::srv::CalculatePoseFromCloud>(name, conf, params)
{}

PortsList CalculatePoseBehaviour::providedPorts()
{
    return providedBasicPorts({
        InputPort<std::map<std::string, sensor_msgs::msg::PointCloud2>>("object_clouds"),
        InputPort<std::string>("object_of_interest"),
        OutputPort<geometry_msgs::msg::PoseStamped>("calculated_pose")
    });
}

bool CalculatePoseBehaviour::setRequest(Request::SharedPtr& request)
{
    std::map<std::string, sensor_msgs::msg::PointCloud2> object_clouds;
    this->getInput("object_clouds", object_clouds);

    std::string object_of_interest;
    this->getInput("object_of_interest", object_of_interest);

    bool object_found = false;
    if (object_clouds.find(object_of_interest) != object_clouds.end())
    {
        request->point_cloud = object_clouds[object_of_interest];
        object_found = true;
    }
    else
    {
        RCLCPP_ERROR(this->logger(), "Received unknown object %s", object_of_interest.c_str());
        for (const auto &kv : object_clouds)
        {
            if (kv.first.find(object_of_interest) != std::string::npos)
            {
                RCLCPP_INFO(this->logger(), "Calculating pose of known object %s", kv.first.c_str());
                request->point_cloud = kv.second;
                object_found = true;
                break;
            }
        }
    }

    return object_found;
}

NodeStatus CalculatePoseBehaviour::onResponseReceived(const Response::SharedPtr& response)
{
    if (response->calculation_successful)
    {
        this->setOutput("calculated_pose", response->pose);
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
}

NodeStatus CalculatePoseBehaviour::onFailure(ServiceNodeErrorCode error)
{
    RCLCPP_ERROR(this->logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
}

RosNodeParams CalculatePoseBehaviour::setCustomParams(RosNodeParams params)
{
    params.server_timeout = std::chrono::milliseconds(10000);
    return params;
}

CreateRosNodePlugin(CalculatePoseBehaviour, "CalculatePoseBehaviour");
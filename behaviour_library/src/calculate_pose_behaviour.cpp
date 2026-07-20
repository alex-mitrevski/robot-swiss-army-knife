#include "behaviour_library/calculate_pose_behaviour.hpp"
#include <behaviortree_ros2/plugins.hpp>

CalculatePoseBehaviour::CalculatePoseBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
: RosServiceNode<robot_swiss_knife_msgs::srv::CalculatePoseFromCloud>(name, conf, params)
{}

PortsList CalculatePoseBehaviour::providedPorts()
{
    return providedBasicPorts({
        InputPort<std::vector<robot_swiss_knife_msgs::msg::Object>>("objects"),
        InputPort<std::string>("object_of_interest"),
        OutputPort<robot_swiss_knife_msgs::msg::Object>("object_with_pose")
    });
}

bool CalculatePoseBehaviour::setRequest(Request::SharedPtr& request)
{
    this->getInput("objects", this->objects);
    this->getInput("object_of_interest", this->object_of_interest);

    bool object_found = false;

    // we check if an object with the given name is in the list
    for (const robot_swiss_knife_msgs::msg::Object& obj : this->objects)
    {
        if (obj.name == this->object_of_interest)
        {
            this->object_calculating_pose_for = obj.name;
            request->point_cloud = obj.view.point_cloud;
            object_found = true;
        }
    }

    // if we don't find an object with the exact name, we look for the
    // first object in the list that has a partial name match
    // (e.g. if the requested name if 'person', but the list has 'person_0',
    //  we will calculate the pose of 'person_0')
    if (!object_found)
    {
        RCLCPP_ERROR(this->logger(), "Received unknown object %s", this->object_of_interest.c_str());
        for (const robot_swiss_knife_msgs::msg::Object& obj : this->objects)
        {
            if (obj.name.find(this->object_of_interest) != std::string::npos)
            {
                RCLCPP_INFO(this->logger(), "Calculating pose of known object %s", obj.name.c_str());
                this->object_calculating_pose_for = obj.name;
                request->point_cloud = obj.view.point_cloud;
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
        for (robot_swiss_knife_msgs::msg::Object& obj : this->objects)
        {
            if (obj.name == this->object_calculating_pose_for)
            {
                obj.pose = response->pose;
                this->setOutput("object_with_pose", obj);
                break;
            }
        }
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
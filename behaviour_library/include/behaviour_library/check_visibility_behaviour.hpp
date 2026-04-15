#ifndef CHECK_VISIBILITY_BEHAVIOUR_HPP
#define CHECK_VISIBILITY_BEHAVIOUR_HPP

#include <vector>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "robot_swiss_knife_msgs/srv/check_object_visibility.hpp"

using namespace BT;

/**
 * A behaviour for interacting with a service of type robot_swiss_knife_msgs::srv::CheckObjectVisibility.
 * Based on the samples in https://github.com/BehaviorTree/BehaviorTree.ROS2
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class CheckVisibilityBehaviour : public RosServiceNode<robot_swiss_knife_msgs::srv::CheckObjectVisibility>
{
public:
    CheckVisibilityBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    /**
     * Sets the ports of the behaviour, which correspond to the service request and response:
     * - Input ports:
     *     * latest_image (sensor_msgs::msg::Image)
     *     * object_categories (std::vector<std::string>)
     * - Output ports:
     *     * visible_objects (std::vector<std::string>)
     */
    static PortsList providedPorts();

    /**
     * Sets the service request based on the blackboard contents.
     */
    virtual bool setRequest(Request::SharedPtr& request) override;

    /**
     * Updates the blackboard based on the received response.
     */
    virtual NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

    /**
     * Registers a node failure.
     */
    virtual NodeStatus onFailure(ServiceNodeErrorCode error) override;
};

#endif
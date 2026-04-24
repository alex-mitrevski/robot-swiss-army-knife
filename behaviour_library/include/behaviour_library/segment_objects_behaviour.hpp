#ifndef SEGMENT_OBJECTS_BEHAVIOUR_HPP
#define SEGMENT_OBJECTS_BEHAVIOUR_HPP

#include <vector>
#include <map>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "robot_swiss_knife_msgs/srv/segment_objects.hpp"

using namespace BT;

/**
 * A behaviour for interacting with a service of type robot_swiss_knife_msgs::srv::SegmentObjects.
 * Based on the samples in https://github.com/BehaviorTree/BehaviorTree.ROS2
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class SegmentObjectsBehaviour : public RosServiceNode<robot_swiss_knife_msgs::srv::SegmentObjects>
{
public:
    SegmentObjectsBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    /**
     * Sets the ports of the behaviour, which correspond to the service request and response:
     * - Input ports:
     *     * latest_image (sensor_msgs::msg::Image)
     *     * object_categories (std::vector<std::string>)
     * - Output ports:
     *     * segmented_objects (std::map<std::string, sensor_msgs::msg::Image>)
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
private:
    RosNodeParams setCustomParams(RosNodeParams params);
};

#endif
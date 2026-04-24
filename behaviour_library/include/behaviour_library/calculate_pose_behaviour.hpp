#ifndef CALCULATE_POSE_BEHAVIOUR_HPP
#define CALCULATE_POSE_BEHAVIOUR_HPP

#include <vector>
#include <map>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "robot_swiss_knife_msgs/srv/calculate_pose_from_cloud.hpp"

using namespace BT;

/**
 * A behaviour for interacting with a service of type robot_swiss_knife_msgs::srv::CalculatePoseFromCloud.
 * Based on the samples in https://github.com/BehaviorTree/BehaviorTree.ROS2
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class CalculatePoseBehaviour : public RosServiceNode<robot_swiss_knife_msgs::srv::CalculatePoseFromCloud>
{
public:
    CalculatePoseBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    /**
     * Sets the ports of the behaviour, which correspond to the service request and response:
     * - Input ports:
     *     * object_clouds (std::map<std::string, sensor_msgs::msg::PointCloud2>)
     *     * object_of_interest (std::string)
     * - Output ports:
     *     * calculated_pose (geometry_msgs::msg::PoseStamped)
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
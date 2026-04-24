#ifndef EXTRACT_ROI_3D_POINTS_BEHAVIOUR_HPP
#define EXTRACT_ROI_3D_POINTS_BEHAVIOUR_HPP

#include <vector>
#include <map>
#include <behaviortree_ros2/bt_service_node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "robot_swiss_knife_msgs/srv/extract_roi3_d_points.hpp"

using namespace BT;

/**
 * A behaviour for interacting with a service of type robot_swiss_knife_msgs::srv::ExtractROI3DPoints.
 * Based on the samples in https://github.com/BehaviorTree/BehaviorTree.ROS2
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class ExtractROI3DPointsBehaviour : public RosServiceNode<robot_swiss_knife_msgs::srv::ExtractROI3DPoints>
{
public:
    ExtractROI3DPointsBehaviour(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    /**
     * Sets the ports of the behaviour, which correspond to the service request and response:
     * - Input ports:
     *     * latest_point_cloud (sensor_msgs::msg::PointCloud2)
     *     * segmented_objects (std::map<std::string, sensor_msgs::msg::Image>)
     * - Output ports:
     *     * object_clouds (std::map<std::string, sensor_msgs::msg::PointCloud2>)
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

    std::vector<std::string> object_names;
};

#endif
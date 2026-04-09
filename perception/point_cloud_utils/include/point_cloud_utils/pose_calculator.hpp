#ifndef POSE_CALCULATOR_HPP
#define POSE_CALCULATOR_HPP

#include <thread>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <robot_swiss_knife_msgs/srv/calculate_pose_from_cloud.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * A component for calculating an object pose given a point cloud.
 * 
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class PoseCalculator : public rclcpp::Node
{
public:
    PoseCalculator();
private:
    /**
     * Service callback that calculates an object's pose from its point cloud.
     * The pose is aligned along the cloud's principal axis.
     *
     * @param request Service request containing an input point cloud
     * @param response Service response containing the calculated pose
     */
    void calculate_pose(const std::shared_ptr<robot_swiss_knife_msgs::srv::CalculatePoseFromCloud::Request> request,
                        std::shared_ptr<robot_swiss_knife_msgs::srv::CalculatePoseFromCloud::Response> response);

    rclcpp::Service<robot_swiss_knife_msgs::srv::CalculatePoseFromCloud>::SharedPtr pose_calculation_server;
    std::string pose_calculation_srv_name;
};

#endif
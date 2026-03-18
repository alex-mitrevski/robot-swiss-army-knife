#ifndef MOVEIT_CARTESIAN_CONTROLLER_HPP
#define MOVEIT_CARTESIAN_CONTROLLER_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;

class MoveitCartesianController : public rclcpp::Node
{
public:
    MoveitCartesianController(const rclcpp::NodeOptions &node_options);

    /**
     * Initialises a move group interface with all launch file parameters.
     * Attempts the initialisation a predefined number of times,
     * just in case MoveIt is still initialising.
     */
    void initialise();

    /**
     * Executes a received trajectory by moving through each of the waypoints
     * in the trajectory. If a point in the trajectory cannot be reached, attempts
     * the next point on the trajectory, until all points have been processed.
     */
    void execute_trajectory();

    /**
     * Returns true if a valid trajectory request has been received and
     * should thus the trajectory should be pursued; returns false otherwise.
     */
    bool execution_request_received();

private:
    /**
     * Registers a new trajectory request. If a trajectory is currently being executed,
     * overwrites the request, so a robot would start following the new trajectory.
     * Ignores the request if the message is empty.
     *
     * @param request_msg A message containing poses through which a robot should move
     */
    void trajectory_request_cb(const geometry_msgs::msg::PoseArray &request_msg);

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr trajectory_request_sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trajectory_execution_result_pub;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    bool new_request_received;
    std_msgs::msg::Bool execution_result_msg;

    /////////////////////////////
    // initialisation parameters
    /////////////////////////////
    int max_moveit_initialisation_attempts;
    std::string move_group_name;
    std::string planner_id;
    int number_of_planning_attempts;
    double max_velocity_scaling_factor;
    double max_acceleration_scaling_factor;
    std::string request_topic;
    std::string result_topic;
};

#endif

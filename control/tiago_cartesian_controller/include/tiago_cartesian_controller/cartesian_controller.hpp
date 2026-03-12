#ifndef TIAGO_CARTESIAN_CONTROLLER_HPP
#define TIAGO_CARTESIAN_CONTROLLER_HPP

#include <thread>
#include <chrono>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <kinematics_interface_kdl/kinematics_interface_kdl.hpp>
#include "utils.hpp"

using namespace std::chrono_literals;
using namespace kinematics_interface_kdl;
using std::placeholders::_1;

/**
 * A component exposing a Cartesian controller for TIAGo's arm.
 * Builds on the ROS control KDL kinematics interface:
 * https://github.com/ros-controls/kinematics_interface/tree/humble/kinematics_interface_kdl
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class CartesianController : public rclcpp::Node
{
public:
    CartesianController(const rclcpp::NodeOptions &node_options);

    void execute_goal();
    bool init_kinematics_interface();
    bool new_goal_received;
private:
    void cartesian_goal_cb(const geometry_msgs::msg::PoseStamped &pose_stamped_msg);
    void joint_state_cb(const sensor_msgs::msg::JointState &joint_state_msg);
    void get_end_effector_pose();
    void stop_motion();

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_trajectory_goal_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr torso_trajectory_goal_pub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cartesian_goal_sub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
    std::shared_ptr<kinematics_interface_kdl::KinematicsInterfaceKDL> kinematics_interface;

    std::string goal_topic;
    std::string arm_joint_controller_topic;
    std::string torso_joint_controller_topic;
    std::string joint_states_topic;
    std::string base_link_frame_name;
    std::string end_effector_frame_name;
    std::vector<std::string> joint_names;
    std::vector<std::string> arm_joint_names;
    std::vector<std::string> torso_joint_names;
    std::vector<double> min_arm_joint_vel_limits;
    std::vector<double> max_arm_joint_vel_limits;

    CartesianPose current_cartesian_goal;
    CartesianPose latest_end_effector_pose;
    sensor_msgs::msg::JointState latest_joint_state_msg;
    std::map<std::string, unsigned int> joint_name_to_idx_map;
    trajectory_msgs::msg::JointTrajectory arm_joint_trajectory_goal;
    trajectory_msgs::msg::JointTrajectory torso_joint_trajectory_goal;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    bool names_to_idx_map_initialised;
    bool executing_goal;
    double goal_threshold_m;
    double goal_threshold_rad;
    bool debug_mode;
};

#endif
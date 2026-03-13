#ifndef HAND_OVER_SKILL_HPP
#define HAND_OVER_SKILL_HPP

#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <thread>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tts_msgs/action/tts.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

#include "robot_swiss_knife_msgs/srv/estimate_anthropometric_parameters.hpp"
#include "hand_over_skill/action/hand_over_skill.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

/**
 * A configurable robot hand-over skill.
 * The skeleton of the skill's lifecycle node was created with PAL's rpk utility.
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class HandOverSkillNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using HandOverSkill = hand_over_skill::action::HandOverSkill;
    using HandOverSkillGoal = std::shared_ptr<const HandOverSkill::Goal>;
    using HandOverSkillGoalHandle = std::shared_ptr<rclcpp_action::ServerGoalHandle<HandOverSkill>>;

    explicit HandOverSkillNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~HandOverSkillNode();

    /**
     * Initialises the skill server as well as all publishers, subscribers, and service proxies in the node.
     */
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &);

    /**
     * Simply returns success so the node can become active.
     */
    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &);

    /**
     * Simply returns success so the node can be deactivated.
     */
    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &);

    /**
     * Takes care of cleaning up the node's components, and returns success.
     */
    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

    /**
     * Takes care of cleaning up the node's components if the node has not been deactivated yet, and returns success.
     */
    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

private:
    /**
     * Disables all publishers in the node and resets internal bookkeeping variables.
     */
    void clean_up();

    /**
     * Takes care of executing the skill, which includes:
     * - detecting the height of the person
     * - moving to a handover position based on the height
     * - waiting for the person to take the object
     *
     * @return Currently "success" regardless of the outcome
     */
    std::string execute_skill();

    /**
     * Accepts the goal if the node is active; rejects the goal otherwise.
     * Both method arguments are unused.
     */
    rclcpp_action::GoalResponse request_goal_cb(const rclcpp_action::GoalUUID &, HandOverSkillGoal);

    /**
     * Takes care of executing the skill and setting the execution result.
     *
     * @param goal_handle Skill goal handle
     */
    void request_accepted_cb(const HandOverSkillGoalHandle goal_handle);

    /**
     * Registers the latest camera image.
     *
     * @param msg Latest camera image
     */
    void image_cb(const sensor_msgs::msg::Image &msg);

    /**
     * Registers the latest wrench measurement. Stores the measurement in a queue
     * of the latest this->max_wrench_measurements measurements, and updates a cumulative
     * force sum (along z) for handover detection.
     *
     * @param wrench_msg Latest wrench measurement
     */
    void wrench_cb(const geometry_msgs::msg::WrenchStamped &wrench_msg);

    /**
     * Registers an arm trajectory execution result.
     *
     * @param msg Latest camera image
     */
    void arm_trajectory_execution_result_cb(const std_msgs::msg::Bool &msg);

    /**
     * Publishes diagnostic results about the skill node.
     */
    void publish_diagnostics();

    /**
     * Waits for a person to pull an object from the robot's gripper.
     * Detects pulling by monitoring for large force deviations along the z axis.
     *
     * @return true if a handover is detected within an alloted time; false otherwise
     */
    bool wait_for_handover();

    /**
     * Sends a speech synthesis request to the robot
     *
     * @param msg Message to say
     */
    void say(const std::string &msg) const;

    rclcpp::OnShutdownCallbackHandle shutdown_cb_handle;
    rclcpp_action::Server<HandOverSkill>::SharedPtr skill_server;
    std::shared_ptr<rclcpp::TimerBase> run_timer;
    std::shared_ptr<rclcpp::TimerBase> diagnostics_timer;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr arm_trajectory_request_pub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr gripper_trajectory_pub;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_trajectory_execution_result_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub;
    rclcpp::CallbackGroup::SharedPtr subscriber_callback_group;
    rclcpp::Client<robot_swiss_knife_msgs::srv::EstimateAnthropometricParameters>::SharedPtr anthropometric_param_estimation_client;
    rclcpp_action::Client<tts_msgs::action::TTS>::SharedPtr say_client;

    std::string skill_name;
    std::string camera_image_topic;
    std::string ft_sensor_topic;
    std::string trajectory_execution_topic;
    std::string trajectory_execution_result_topic;
    std::string gripper_trajectory_topic;
    std::string anthropometric_parameter_estimation_srv_name;
    std::string say_action_name;
    sensor_msgs::msg::Image latest_image;
    bool estimate_anthropometric_parameters;
    bool arm_trajectory_execution_result_received;
    bool arm_trajectory_execution_successful;
    std::vector<std::string> gripper_joint_names;
    std::vector<double> gripper_joint_opening_angles;

    /////////////////////////////////////////////////////
    // variables used for force-based handover detection
    /////////////////////////////////////////////////////
    std::deque<geometry_msgs::msg::WrenchStamped> wrench_queue;
    unsigned int min_wrench_measurements_for_handover_detection;
    unsigned int max_wrench_measurements;
    unsigned int wrench_filter_window_size;
    double handover_force_threshold_N;
    double handover_timeout_s;
    double wrench_cusum;
};

#endif

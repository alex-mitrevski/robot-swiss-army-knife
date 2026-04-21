#ifndef MOVE_BASE_SKILL_HPP
#define MOVE_BASE_SKILL_HPP

#include <thread>
#include <string>
#include <chrono>
#include <functional>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tts_msgs/action/tts.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "move_base_skill/action/move_base_skill.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

/**
 * A skill for moving a robot's base between locations.
 * The skill is a high-level interface to a robot's own navigation interface,
 * namely target goals are sent to an action exposed by the robot's navigation stack.
 * The skeleton of the skill's lifecycle node was created with PAL's rpk utility.
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class MoveBaseSkillNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using MoveBaseSkill = move_base_skill::action::MoveBaseSkill;
    using MoveBaseSkillGoal = std::shared_ptr<const MoveBaseSkill::Goal>;
    using MoveBaseSkillGoalHandle = std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveBaseSkill>>;

    explicit MoveBaseSkillNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~MoveBaseSkillNode();

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
     * Takes care of executing the skill by sending the target poses
     * one by one to the robot's move_base executor. Currently only
     * supports goals of type POSE.
     *
     * @return "success" if the robot can move between poses, false otherwise
     */
    void execute_skill(const MoveBaseSkillGoalHandle);

    /**
     * Monitors whether the navigation goal has been completed.
     * The monitoring is stopped if a cancellation request has been received.
     *
     * @param skill_goal_handle Skill goal handle
     * @param timeout_s Timeout before giving up on the goal
     */
    void wait_until_navigation_complete(const MoveBaseSkillGoalHandle skill_goal_handle, double timeout_s);

    /**
     * Cancels ongoing navigation tasks.
     */
    void cancel_navigation();

    /**
     * Accepts the goal if the node is active; rejects the goal otherwise.
     * Both method arguments are unused.
     */
    rclcpp_action::GoalResponse request_goal_cb(const rclcpp_action::GoalUUID &, MoveBaseSkillGoal);

    /**
     * Takes care of executing the skill and setting the execution result.
     *
     * @param goal_handle Skill goal handle
     */
    void request_accepted_cb(const MoveBaseSkillGoalHandle goal_handle);

    /**
     * Registers a cancellation request.
     *
     * @param goal_handle Skill goal handle
     */
    rclcpp_action::CancelResponse cancel_cb(const MoveBaseSkillGoalHandle goal_handle);

    /**
     * Publishes diagnostic results about the skill node.
     */
    void publish_diagnostics();

    /**
     * Sends a navigation request to the robot
     *
     * @param pose Navigation goal pose
     */
    void send_nav_goal(const geometry_msgs::msg::PoseStamped &pose) const;

    /**
     * Sends a speech synthesis request to the robot
     *
     * @param msg Message to say
     */
    void say(const std::string &msg) const;

    void nav_goal_response_cb(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle);

    void nav_feedback_cb(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
                         const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

    void nav_result_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result);

    rclcpp::OnShutdownCallbackHandle shutdown_cb_handle;
    rclcpp_action::Server<MoveBaseSkill>::SharedPtr skill_server;
    std::shared_ptr<rclcpp::TimerBase> run_timer;
    std::shared_ptr<rclcpp::TimerBase> diagnostics_timer;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions nav_goal_options;
    rclcpp_action::Client<tts_msgs::action::TTS>::SharedPtr say_client;
    rclcpp::CallbackGroup::SharedPtr reentrant_callback_group;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_goal_handle;

    std::string skill_name;
    std::string nav_action_name;
    std::string say_action_name;
    std::string base_link_frame_name;
    std::string map_frame_name;
    bool navigation_complete;
    bool goal_reached;
    double max_pose_timeout_s;
    bool debug_mode;
};

#endif

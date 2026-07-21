#include "pick_skill/pick_skill.hpp"

PickSkillNode::PickSkillNode(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("pick_skill", "", options), skill_name("pick_skill")
{
    RCLCPP_INFO(this->get_logger(), "[%s] Initialising...", this->skill_name.c_str());

    this->declare_parameter("camera_image_topic", "/xtion/rgb/image_raw");
    this->declare_parameter("trajectory_execution_topic", "/get_trajectory");
    this->declare_parameter("trajectory_execution_result_topic", "/trajectory_execution_result");
    this->declare_parameter("gripper_trajectory_topic", "/gripper_controller/joint_trajectory");
    this->declare_parameter("ft_sensor_topic", "/ft_sensor_controller/wrench");
    this->declare_parameter("say_action_name", "/tts_engine/tts");
    this->declare_parameter("min_wrench_measurements_for_grasp_detection", 10);
    this->declare_parameter("max_wrench_measurements", 100);
    this->declare_parameter("wrench_filter_window_size", 5);
    this->declare_parameter("gripper_joint_names", std::vector<std::string>({"gripper_left_finger_joint", "gripper_right_finger_joint"}));
    this->declare_parameter("gripper_joint_opening_angles", std::vector<double>({0.025, 0.045}));
    this->declare_parameter("gripper_joint_closing_angles", std::vector<double>({0.02, 0.02}));

    this->get_parameter("camera_image_topic", this->camera_image_topic);
    this->get_parameter("trajectory_execution_topic", this->trajectory_execution_topic);
    this->get_parameter("trajectory_execution_result_topic", this->trajectory_execution_result_topic);
    this->get_parameter("gripper_trajectory_topic", this->gripper_trajectory_topic);
    this->get_parameter("ft_sensor_topic", this->ft_sensor_topic);
    this->get_parameter("say_action_name", this->say_action_name);
    this->min_wrench_measurements_for_grasp_detection = this->get_parameter("min_wrench_measurements_for_grasp_detection").as_int();
    this->max_wrench_measurements = this->get_parameter("max_wrench_measurements").as_int();
    this->wrench_filter_window_size = this->get_parameter("wrench_filter_window_size").as_int();
    this->get_parameter("gripper_joint_names", this->gripper_joint_names);
    this->get_parameter("gripper_joint_opening_angles", this->gripper_joint_opening_angles);
    this->get_parameter("gripper_joint_closing_angles", this->gripper_joint_closing_angles);

    this->arm_trajectory_execution_result_received = false;
    this->arm_trajectory_execution_successful = false;
    this->shutdown_cb_handle = this->get_node_options().context()->add_on_shutdown_callback([this]() {this->on_shutdown(this->get_current_state());});

    RCLCPP_INFO(this->get_logger(), "[%s] Started skill, but not yet configured", this->skill_name.c_str());
}

PickSkillNode::~PickSkillNode()
{
    this->get_node_options().context()->remove_on_shutdown_callback(this->shutdown_cb_handle);
}

PickSkillNode::LifecycleCallbackReturn PickSkillNode::on_configure(const rclcpp_lifecycle::State &)
{
    this->skill_server = rclcpp_action::create_server<PickSkill>(this, "/skill/pick",
                                                                     std::bind(&PickSkillNode::request_goal_cb, this, _1, _2),
                                                                     nullptr,
                                                                     std::bind(&PickSkillNode::request_accepted_cb, this, _1));
    this->arm_trajectory_request_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(this->trajectory_execution_topic, 10);
    this->gripper_trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(this->gripper_trajectory_topic, 10);
    this->diagnostics_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
    this->diagnostics_timer = rclcpp::create_timer(this, this->get_clock(),
                                                   std::chrono::seconds(1),
                                                   std::bind(&PickSkillNode::publish_diagnostics, this));

    this->subscriber_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions reentrant_group_options;
    reentrant_group_options.callback_group = this->subscriber_callback_group;

    this->say_client = rclcpp_action::create_client<tts_msgs::action::TTS>(this, this->say_action_name);

    this->arm_trajectory_execution_result_sub = this->create_subscription<std_msgs::msg::Bool>(this->trajectory_execution_result_topic,
                                                                                               10,
                                                                                               std::bind(&PickSkillNode::arm_trajectory_execution_result_cb, this, _1),
                                                                                               reentrant_group_options);
    this->image_sub = this->create_subscription<sensor_msgs::msg::Image>(this->camera_image_topic,
                                                                         rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile(),
                                                                         std::bind(&PickSkillNode::image_cb, this, _1),
                                                                         reentrant_group_options);
    this->wrench_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(this->ft_sensor_topic,
                                                                                    10,
                                                                                    std::bind(&PickSkillNode::wrench_cb, this, _1),
                                                                                    reentrant_group_options);

    RCLCPP_INFO(this->get_logger(), "[%s] Configuration complete",  this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

PickSkillNode::LifecycleCallbackReturn PickSkillNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "[%s] Skill is active and running", this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

PickSkillNode::LifecycleCallbackReturn PickSkillNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "[%s] Skill has been stopped", this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

PickSkillNode::LifecycleCallbackReturn PickSkillNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    this->clean_up();
    RCLCPP_INFO(this->get_logger(), "[%s] Skill has been cleaned up", this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

PickSkillNode::LifecycleCallbackReturn PickSkillNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
    if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        this->clean_up();
    }
    else if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        this->clean_up();
    }
    RCLCPP_INFO(this->get_logger(), "[%s] Skill shutdown", this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

void PickSkillNode::clean_up()
{
    this->arm_trajectory_request_pub.reset();
    this->gripper_trajectory_pub.reset();
    this->diagnostics_timer.reset();
    this->diagnostics_pub.reset();
    this->skill_server.reset();
    this->wrench_queue.clear();
    this->wrench_cusum = 0.0;
}

void PickSkillNode::execute_skill(const PickSkillGoalHandle goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "[%s] Running skill", this->skill_name.c_str());
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PickSkill::Result>();

    trajectory_msgs::msg::JointTrajectory gripper_trajectory;
    trajectory_msgs::msg::JointTrajectoryPoint gripper_goal;
    gripper_trajectory.joint_names = this->gripper_joint_names;

    RCLCPP_INFO(this->get_logger(), "[%s] Opening gripper", this->skill_name.c_str());
    gripper_goal.positions = this->gripper_joint_opening_angles;
    gripper_trajectory.points.push_back(gripper_goal);
    this->gripper_trajectory_pub->publish(gripper_trajectory);

    geometry_msgs::msg::PoseArray trajectory_execution_request_msg;
    trajectory_execution_request_msg.header.frame_id = goal->object.pose.header.frame_id;
    trajectory_execution_request_msg.poses.push_back(goal->object.pose.pose);

    RCLCPP_INFO(this->get_logger(), "[%s] Going to pickup pose: (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f, %.2f)",
                this->skill_name.c_str(), goal->object.pose.pose.position.x, goal->object.pose.pose.position.y, goal->object.pose.pose.position.z,
                goal->object.pose.pose.orientation.x, goal->object.pose.pose.orientation.y, goal->object.pose.pose.orientation.z, goal->object.pose.pose.orientation.w);
    this->say("I will move towards the object");

    this->arm_trajectory_execution_result_received = false;
    this->arm_trajectory_request_pub->publish(trajectory_execution_request_msg);

    // we wait for a while so that the arm has time to reach execute the trajectory
    rclcpp::Rate rate(5);
    while (!this->arm_trajectory_execution_result_received)
    {
        rate.sleep();
    }

    if (this->arm_trajectory_execution_successful)
    {
        RCLCPP_INFO(this->get_logger(), "[%s] Successfully generated arm trajectory", this->skill_name.c_str());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "[%s] Failed to generate arm trajectory", this->skill_name.c_str());
    }

    gripper_goal.positions = this->gripper_joint_closing_angles;
    gripper_trajectory.points.push_back(gripper_goal);
    this->gripper_trajectory_pub->publish(gripper_trajectory);

    RCLCPP_INFO(this->get_logger(), "[%s] Skill execution complete", this->skill_name.c_str());
    goal_handle->succeed(result);
}

rclcpp_action::GoalResponse PickSkillNode::request_goal_cb(const rclcpp_action::GoalUUID &, PickSkillGoal)
{
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s] Skill is not active yet, rejecting goal", this->skill_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "[%s] Accepted a new goal", this->skill_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void PickSkillNode::request_accepted_cb(const PickSkillGoalHandle goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "[%s] Executing skill", this->skill_name.c_str());
    std::thread{std::bind(&PickSkillNode::execute_skill, this, _1), goal_handle}.detach();
}

void PickSkillNode::publish_diagnostics()
{
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = "/skill/pick";
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "skill pick is running");
    status.add("lifecycle-state", this->get_current_state().label());

    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = this->get_clock()->now();
    msg.status.push_back(status);
    this->diagnostics_pub->publish(msg);
}

void PickSkillNode::image_cb(const sensor_msgs::msg::Image &msg)
{
    this->latest_image = msg;
}

void PickSkillNode::wrench_cb(const geometry_msgs::msg::WrenchStamped &wrench_msg)
{
    if (this->wrench_queue.size() == this->max_wrench_measurements)
    {
        this->wrench_queue.pop_front();
    }

    if (this->wrench_queue.size() != 0)
    {
        auto last_measurement = this->wrench_queue.back();
        this->wrench_cusum += (wrench_msg.wrench.force.z - last_measurement.wrench.force.z);
    }
    this->wrench_queue.push_back(wrench_msg);
}

void PickSkillNode::arm_trajectory_execution_result_cb(const std_msgs::msg::Bool &msg)
{
    this->arm_trajectory_execution_result_received = true;
    this->arm_trajectory_execution_successful = msg.data;
}

void PickSkillNode::say(const std::string &msg) const
{
    auto say_goal = tts_msgs::action::TTS::Goal();
    say_goal.locale = "en_US";
    say_goal.input = msg;
    this->say_client->async_send_goal(say_goal);
}
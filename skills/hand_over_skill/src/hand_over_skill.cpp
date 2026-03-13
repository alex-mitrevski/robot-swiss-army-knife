#include "hand_over_skill/hand_over_skill.hpp"

HandOverSkillNode::HandOverSkillNode(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("hand_over_skill", "", options), skill_name("hand_over_skill")
{
    RCLCPP_INFO(this->get_logger(), "[%s] Initialising...", this->skill_name.c_str());

    this->declare_parameter("camera_image_topic", "/xtion/rgb/image_raw");
    this->declare_parameter("trajectory_execution_topic", "/get_trajectory");
    this->declare_parameter("trajectory_execution_result_topic", "/trajectory_execution_result");
    this->declare_parameter("gripper_trajectory_topic", "/gripper_controller/joint_trajectory");
    this->declare_parameter("ft_sensor_topic", "/ft_sensor_controller/wrench");
    this->declare_parameter("estimate_anthropometric_parameters", true);
    this->declare_parameter("anthropometric_parameter_estimation_srv", "estimate_anthropometric_parameters");
    this->declare_parameter("say_action_name", "/tts_engine/tts");
    this->declare_parameter("min_wrench_measurements_for_handover_detection", 10);
    this->declare_parameter("max_wrench_measurements", 100);
    this->declare_parameter("wrench_filter_window_size", 5);
    this->declare_parameter("handover_force_threshold_N", 10.0);
    this->declare_parameter("handover_timeout_s", 20.0);
    this->declare_parameter("gripper_joint_names", std::vector<std::string>({"gripper_left_finger_joint", "gripper_right_finger_joint"}));
    this->declare_parameter("gripper_joint_opening_angles", std::vector<double>({0.025, 0.045}));

    this->get_parameter("camera_image_topic", this->camera_image_topic);
    this->get_parameter("trajectory_execution_topic", this->trajectory_execution_topic);
    this->get_parameter("trajectory_execution_result_topic", this->trajectory_execution_result_topic);
    this->get_parameter("gripper_trajectory_topic", this->gripper_trajectory_topic);
    this->get_parameter("ft_sensor_topic", this->ft_sensor_topic);
    this->get_parameter("estimate_anthropometric_parameters", this->estimate_anthropometric_parameters);
    this->get_parameter("anthropometric_parameter_estimation_srv", this->anthropometric_parameter_estimation_srv_name);
    this->get_parameter("say_action_name", this->say_action_name);
    this->min_wrench_measurements_for_handover_detection = this->get_parameter("min_wrench_measurements_for_handover_detection").as_int();
    this->max_wrench_measurements = this->get_parameter("max_wrench_measurements").as_int();
    this->wrench_filter_window_size = this->get_parameter("wrench_filter_window_size").as_int();
    this->get_parameter("handover_force_threshold_N", this->handover_force_threshold_N);
    this->get_parameter("handover_timeout_s", this->handover_timeout_s);
    this->get_parameter("gripper_joint_names", this->gripper_joint_names);
    this->get_parameter("gripper_joint_opening_angles", this->gripper_joint_opening_angles);

    this->arm_trajectory_execution_result_received = false;
    this->arm_trajectory_execution_successful = false;
    this->shutdown_cb_handle = this->get_node_options().context()->add_on_shutdown_callback([this]() {this->on_shutdown(this->get_current_state());});

    RCLCPP_INFO(this->get_logger(), "[%s] Started skill, but not yet configured", this->skill_name.c_str());
}

HandOverSkillNode::~HandOverSkillNode()
{
    this->get_node_options().context()->remove_on_shutdown_callback(this->shutdown_cb_handle);
}

HandOverSkillNode::LifecycleCallbackReturn HandOverSkillNode::on_configure(const rclcpp_lifecycle::State &)
{
    this->skill_server = rclcpp_action::create_server<HandOverSkill>(this, "/skill/hand_over",
                                                                     std::bind(&HandOverSkillNode::request_goal_cb, this, _1, _2),
                                                                     nullptr,
                                                                     std::bind(&HandOverSkillNode::request_accepted_cb, this, _1));
    this->arm_trajectory_request_pub = this->create_publisher<geometry_msgs::msg::PoseArray>(this->trajectory_execution_topic, 10);
    this->gripper_trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(this->gripper_trajectory_topic, 10);
    this->diagnostics_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
    this->diagnostics_timer = rclcpp::create_timer(this, this->get_clock(),
                                                   std::chrono::seconds(1),
                                                   std::bind(&HandOverSkillNode::publish_diagnostics, this));

    this->subscriber_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions reentrant_group_options;
    reentrant_group_options.callback_group = this->subscriber_callback_group;

    if (this->estimate_anthropometric_parameters)
    {
        this->anthropometric_param_estimation_client = this->create_client<robot_swiss_knife_msgs::srv::EstimateAnthropometricParameters>(this->anthropometric_parameter_estimation_srv_name,
         rmw_qos_profile_services_default,
         this->subscriber_callback_group);
        while (!this->anthropometric_param_estimation_client->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "[%s] Service %s not available, waiting again",
                        this->skill_name.c_str(), this->anthropometric_parameter_estimation_srv_name.c_str());
        }
    }

    this->say_client = rclcpp_action::create_client<tts_msgs::action::TTS>(this, this->say_action_name);

    this->arm_trajectory_execution_result_sub = this->create_subscription<std_msgs::msg::Bool>(this->trajectory_execution_result_topic,
                                                                                               10,
                                                                                               std::bind(&HandOverSkillNode::arm_trajectory_execution_result_cb, this, _1),
                                                                                               reentrant_group_options);
    this->image_sub = this->create_subscription<sensor_msgs::msg::Image>(this->camera_image_topic,
                                                                         rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile(),
                                                                         std::bind(&HandOverSkillNode::image_cb, this, _1),
                                                                         reentrant_group_options);
    this->wrench_sub = this->create_subscription<geometry_msgs::msg::WrenchStamped>(this->ft_sensor_topic,
                                                                                    10,
                                                                                    std::bind(&HandOverSkillNode::wrench_cb, this, _1),
                                                                                    reentrant_group_options);

    RCLCPP_INFO(this->get_logger(), "[%s] Configuration complete",  this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

HandOverSkillNode::LifecycleCallbackReturn HandOverSkillNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "[%s] Skill is active and running", this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

HandOverSkillNode::LifecycleCallbackReturn HandOverSkillNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "[%s] Skill has been stopped", this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

HandOverSkillNode::LifecycleCallbackReturn HandOverSkillNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    this->clean_up();
    RCLCPP_INFO(this->get_logger(), "[%s] Skill has been cleaned up", this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

HandOverSkillNode::LifecycleCallbackReturn HandOverSkillNode::on_shutdown(const rclcpp_lifecycle::State & state)
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

void HandOverSkillNode::clean_up()
{
    this->arm_trajectory_request_pub.reset();
    this->gripper_trajectory_pub.reset();
    this->diagnostics_timer.reset();
    this->diagnostics_pub.reset();
    this->skill_server.reset();
    this->wrench_queue.clear();
    this->wrench_cusum = 0.0;
}

std::string HandOverSkillNode::execute_skill()
{
    RCLCPP_INFO(this->get_logger(), "[%s] Running skill", this->skill_name.c_str());

    float height_cm;
    float mass_kg;
    if (this->estimate_anthropometric_parameters)
    {
        auto request = std::make_shared<robot_swiss_knife_msgs::srv::EstimateAnthropometricParameters::Request>();
        request->image = this->latest_image;

        RCLCPP_INFO(this->get_logger(), "[%s] Estimating height and mass...", this->skill_name.c_str());
        this->say("I am estimating your height so I can give you the object comfortably. Please wait.");
        auto future = this->anthropometric_param_estimation_client->async_send_request(request);

        if (future.wait_for(std::chrono::seconds(120)) == std::future_status::ready)
        {
            auto response = future.get();
            height_cm = response->height_cm;
            mass_kg = response->mass_kg;
            RCLCPP_INFO(this->get_logger(), "[%s] Estimated height of %.2f cm and mass of %.2f kg",
                        this->skill_name.c_str(), height_cm, mass_kg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "[%s] Failed to call service %s",
                        this->skill_name.c_str(),
                        this->anthropometric_parameter_estimation_srv_name.c_str());
        }
    }
    else
    {
        height_cm = 175.0f;
        mass_kg = 70.0f;
        RCLCPP_INFO(this->get_logger(), "[%s] Not estimating height and mass; using default hand-over values (height=%.2f, mass=%.2f)",
                    this->skill_name.c_str(), height_cm, mass_kg);
    }

    float height_m = height_cm / 100.0f;
    geometry_msgs::msg::Pose hand_over_pose;

    // we set the position based on the person's height
    hand_over_pose.position.x = 0.7;
    hand_over_pose.position.y = 0.1;
    hand_over_pose.position.z =  (height_m / 2.0f) + height_m * 0.1f;

    // we set a fixed hand-over orientation
    hand_over_pose.orientation.x = -0.679;
    hand_over_pose.orientation.y = 0.093;
    hand_over_pose.orientation.z = -0.098;
    hand_over_pose.orientation.w = 0.722;

    geometry_msgs::msg::PoseArray trajectory_execution_request_msg;
    trajectory_execution_request_msg.header.frame_id = "base_link";
    trajectory_execution_request_msg.poses.push_back(hand_over_pose);

    RCLCPP_INFO(this->get_logger(), "[%s] Going to handover pose: (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f, %.2f)",
                this->skill_name.c_str(), hand_over_pose.position.x, hand_over_pose.position.y, hand_over_pose.position.z,
                hand_over_pose.orientation.x, hand_over_pose.orientation.y, hand_over_pose.orientation.z, hand_over_pose.orientation.w);
    this->say("I am moving to a suitable handover position. Please wait.");

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

    this->say("Take the object please.");
    bool handover_detected = this->wait_for_handover();
    if (handover_detected)
    {
        RCLCPP_INFO(this->get_logger(), "[%s] Handover detected; releasing object", this->skill_name.c_str());
        this->say("I am releasing the object. Thank you.");
        trajectory_msgs::msg::JointTrajectory gripper_trajectory;
        gripper_trajectory.joint_names = this->gripper_joint_names;

        trajectory_msgs::msg::JointTrajectoryPoint gripper_goal;
        gripper_goal.positions = this->gripper_joint_opening_angles;

        gripper_trajectory.points.push_back(gripper_goal);
        this->gripper_trajectory_pub->publish(gripper_trajectory);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "[%s] Object not picked up by the user; giving up", this->skill_name.c_str());
        this->say("I am giving up since you are not taking the object.");
    }
    return "success";
}

rclcpp_action::GoalResponse HandOverSkillNode::request_goal_cb(const rclcpp_action::GoalUUID &, HandOverSkillGoal)
{
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s] Skill is not active yet, rejecting goal", this->skill_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "[%s] Accepted a new goal", this->skill_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void HandOverSkillNode::request_accepted_cb(const HandOverSkillGoalHandle goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "[%s] Executing skill", this->skill_name.c_str());

    auto result = std::make_shared<HandOverSkill::Result>();
    result->value = this->execute_skill();
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "[%s] Goal executed successfully", this->skill_name.c_str());
}

void HandOverSkillNode::publish_diagnostics()
{
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = "/skill/hand_over";
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "skill hand_over is running");
    status.add("lifecycle-state", this->get_current_state().label());

    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = this->get_clock()->now();
    msg.status.push_back(status);
    this->diagnostics_pub->publish(msg);
}

bool HandOverSkillNode::wait_for_handover()
{
    RCLCPP_INFO(this->get_logger(), "[%s] Initialising handover detection", this->skill_name.c_str());
    this->wrench_queue.clear();
    this->wrench_cusum = 0.0;
    bool handover_detected = false;
    bool handover_timeout_reached = false;
    rclcpp::Rate sleep_rate(std::chrono::milliseconds(100));
    rclcpp::Time start_time = this->get_clock()->now();
    rclcpp::Duration elapsed_time = this->get_clock()->now() - start_time;
    while (!handover_detected && !handover_timeout_reached)
    {
        handover_detected = abs(this->wrench_cusum) > handover_force_threshold_N;
        if (!handover_detected && elapsed_time.seconds() > this->handover_timeout_s)
        {
            handover_timeout_reached = true;
        }
        sleep_rate.sleep();
    }
    return handover_detected;
}

void HandOverSkillNode::image_cb(const sensor_msgs::msg::Image &msg)
{
    this->latest_image = msg;
}

void HandOverSkillNode::wrench_cb(const geometry_msgs::msg::WrenchStamped &wrench_msg)
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

void HandOverSkillNode::arm_trajectory_execution_result_cb(const std_msgs::msg::Bool &msg)
{
    this->arm_trajectory_execution_result_received = true;
    this->arm_trajectory_execution_successful = msg.data;
}

void HandOverSkillNode::say(const std::string &msg) const
{
    auto say_goal = tts_msgs::action::TTS::Goal();
    say_goal.locale = "en_US";
    say_goal.input = msg;
    this->say_client->async_send_goal(say_goal);
}
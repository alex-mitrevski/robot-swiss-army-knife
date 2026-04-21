#include "move_base_skill/move_base_skill.hpp"

MoveBaseSkillNode::MoveBaseSkillNode(const rclcpp::NodeOptions & options)
    : rclcpp_lifecycle::LifecycleNode("move_base_skill", "", options), skill_name("move_base_skill")
{
    RCLCPP_INFO(this->get_logger(), "[%s] Initialising...", this->skill_name.c_str());

    this->declare_parameter("nav_action_name", "/navigate_to_pose");
    this->declare_parameter("say_action_name", "/tts_engine/tts");
    this->declare_parameter("base_link_frame_name", "base_footprint");
    this->declare_parameter("map_frame_name", "map");
    this->declare_parameter("max_pose_timeout_s", 30.0);
    this->declare_parameter("debug_mode", false);

    this->get_parameter("nav_action_name", this->nav_action_name);
    this->get_parameter("say_action_name", this->say_action_name);
    this->get_parameter("base_link_frame_name", this->base_link_frame_name);
    this->get_parameter("map_frame_name", this->map_frame_name);
    this->get_parameter("max_pose_timeout_s", this->max_pose_timeout_s);
    this->get_parameter("debug_mode", this->debug_mode);

    this->shutdown_cb_handle = this->get_node_options().context()->add_on_shutdown_callback([this]() {this->on_shutdown(this->get_current_state());});

    RCLCPP_INFO(this->get_logger(), "[%s] Started skill, but not yet configured", this->skill_name.c_str());
}

MoveBaseSkillNode::~MoveBaseSkillNode()
{
    this->get_node_options().context()->remove_on_shutdown_callback(this->shutdown_cb_handle);
}

MoveBaseSkillNode::LifecycleCallbackReturn MoveBaseSkillNode::on_configure(const rclcpp_lifecycle::State &)
{
    this->skill_server = rclcpp_action::create_server<MoveBaseSkill>(this, "/skill/move_base",
                                                                     std::bind(&MoveBaseSkillNode::request_goal_cb, this, _1, _2),
                                                                     std::bind(&MoveBaseSkillNode::cancel_cb, this, _1),
                                                                     std::bind(&MoveBaseSkillNode::request_accepted_cb, this, _1));
    this->diagnostics_pub = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 1);
    this->diagnostics_timer = rclcpp::create_timer(this, this->get_clock(),
                                                   std::chrono::seconds(1),
                                                   std::bind(&MoveBaseSkillNode::publish_diagnostics, this));

    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);

    this->reentrant_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    this->nav_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this,
                                                                                       this->nav_action_name,
                                                                                       this->reentrant_callback_group);
    this->say_client = rclcpp_action::create_client<tts_msgs::action::TTS>(this, this->say_action_name);

    this->nav_goal_options.goal_response_callback = std::bind(&MoveBaseSkillNode::nav_goal_response_cb, this, _1);
    this->nav_goal_options.feedback_callback = std::bind(&MoveBaseSkillNode::nav_feedback_cb, this, _1, _2);
    this->nav_goal_options.result_callback = std::bind(&MoveBaseSkillNode::nav_result_cb, this, _1);

    RCLCPP_INFO(this->get_logger(), "[%s] Configuration complete",  this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

MoveBaseSkillNode::LifecycleCallbackReturn MoveBaseSkillNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "[%s] Skill is active and running", this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

MoveBaseSkillNode::LifecycleCallbackReturn MoveBaseSkillNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "[%s] Skill has been stopped", this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

MoveBaseSkillNode::LifecycleCallbackReturn MoveBaseSkillNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    this->clean_up();
    RCLCPP_INFO(this->get_logger(), "[%s] Skill has been cleaned up", this->skill_name.c_str());
    return LifecycleCallbackReturn::SUCCESS;
}

MoveBaseSkillNode::LifecycleCallbackReturn MoveBaseSkillNode::on_shutdown(const rclcpp_lifecycle::State & state)
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

void MoveBaseSkillNode::clean_up()
{
    if (this->nav2_goal_handle)
    {
        this->nav_client->async_cancel_goal(this->nav2_goal_handle);
    }
    this->nav2_goal_handle = nullptr;
    this->diagnostics_timer.reset();
    this->diagnostics_pub.reset();
    this->skill_server.reset();
}

void MoveBaseSkillNode::execute_skill(const MoveBaseSkillGoalHandle goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "[%s] Running skill", this->skill_name.c_str());
    auto goal = goal_handle->get_goal();
    auto result = std::make_shared<MoveBaseSkill::Result>();

    if (goal->goal_type == MoveBaseSkill::Goal::POSE)
    {
        for (geometry_msgs::msg::PoseStamped pose : goal->poses)
        {
            if (goal_handle->is_canceling())
            {
                this->cancel_navigation();
                result->value = "Execution cancelled";
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "[%s] Execution cancelled", this->skill_name.c_str());
                return;
            }

            RCLCPP_INFO(this->get_logger(), "[%s] Going to pose: (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f, %.2f) in frame %s",
                        this->skill_name.c_str(), pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
                        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w, pose.header.frame_id.c_str());

            geometry_msgs::msg::PoseStamped target_pose_in_map_frame;
            try
            {
                target_pose_in_map_frame = this->tf_buffer->transform(pose, this->map_frame_name);
            }
            catch (const tf2::TransformException & ex)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Could not transform %s to %s: %s",
                            this->map_frame_name.c_str(),
                            pose.header.frame_id.c_str(),
                            ex.what());
            }

            this->say("I will move now, please be careful");
            this->navigation_complete = false;
            this->goal_reached = false;
            this->send_nav_goal(target_pose_in_map_frame);
            this->wait_until_navigation_complete(goal_handle, this->max_pose_timeout_s);
            if (!this->goal_reached)
            {
                result->value = "Not all goals could be reached successfully";
            }
        }

        // we take the status of the last goal as final skill status
        result->success = this->goal_reached;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "[%s] Unknown or unsupported goal type %d received; ignoring request", this->skill_name.c_str(), goal->goal_type);
        result->success = false;
        result->value = "Received unknown or unsupported goal type";
        this->say("Cannot move currently");
    }

    RCLCPP_INFO(this->get_logger(), "[%s] Skill execution complete", this->skill_name.c_str());
    goal_handle->succeed(result);
}

void MoveBaseSkillNode::wait_until_navigation_complete(const MoveBaseSkillGoalHandle skill_goal_handle, double timeout_s)
{
    rclcpp::Time start_time = this->get_clock()->now();
    rclcpp::Duration elapsed_time = this->get_clock()->now() - start_time;

    RCLCPP_INFO(this->get_logger(), "[%s] Waiting for navigation goal to be reached", this->skill_name.c_str());
    rclcpp::Rate sleep_rate(std::chrono::milliseconds(100));
    while (!this->navigation_complete && (elapsed_time.seconds() < timeout_s))
    {
        sleep_rate.sleep();
        elapsed_time = this->get_clock()->now() - start_time;

        // if a cancellation request has been received, we
        // stop waiting and pass the control over to the caller
        if (skill_goal_handle->is_canceling())
        {
            this->navigation_complete = true;
        }
    }
}

void MoveBaseSkillNode::cancel_navigation()
{
    if (this->nav2_goal_handle)
    {
        this->nav_client->async_cancel_goal(this->nav2_goal_handle);
    }
}

rclcpp_action::GoalResponse MoveBaseSkillNode::request_goal_cb(const rclcpp_action::GoalUUID &, MoveBaseSkillGoal)
{
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s] Skill is not active yet, rejecting goal", this->skill_name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "[%s] Accepted a new goal", this->skill_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void MoveBaseSkillNode::request_accepted_cb(const MoveBaseSkillGoalHandle goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "[%s] Executing skill", this->skill_name.c_str());
    std::thread{std::bind(&MoveBaseSkillNode::execute_skill, this, _1), goal_handle}.detach();
}

rclcpp_action::CancelResponse MoveBaseSkillNode::cancel_cb(const MoveBaseSkillGoalHandle goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "[%s] Received cancellation request; cancelling goals", this->skill_name.c_str());
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveBaseSkillNode::publish_diagnostics()
{
    diagnostic_updater::DiagnosticStatusWrapper status;
    status.name = "/skill/move_base";
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "skill move_base is running");
    status.add("lifecycle-state", this->get_current_state().label());

    diagnostic_msgs::msg::DiagnosticArray msg;
    msg.header.stamp = this->get_clock()->now();
    msg.status.push_back(status);
    this->diagnostics_pub->publish(msg);
}

void MoveBaseSkillNode::send_nav_goal(const geometry_msgs::msg::PoseStamped &pose) const
{
    auto nav_goal = nav2_msgs::action::NavigateToPose::Goal();
    nav_goal.pose = pose;
    this->nav_client->async_send_goal(nav_goal, this->nav_goal_options);
}

void MoveBaseSkillNode::say(const std::string &msg) const
{
    auto say_goal = tts_msgs::action::TTS::Goal();
    say_goal.locale = "en_US";
    say_goal.input = msg;
    this->say_client->async_send_goal(say_goal);
}

void MoveBaseSkillNode::nav_goal_response_cb(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle)
{
    this->nav2_goal_handle = goal_handle;
    if (!this->nav2_goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s] Navigation goal not accepted by server", this->skill_name.c_str());
        this->navigation_complete = true;
    }
}

void MoveBaseSkillNode::nav_feedback_cb(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
                                        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    if (this->debug_mode)
    {
        RCLCPP_INFO(this->get_logger(), "[%s] Current pose: (%.2f, %.2f, %.2f), (%.2f, %.2f, %.2f, %.2f) in frame %s",
                    this->skill_name.c_str(), feedback->current_pose.pose.position.x, feedback->current_pose.pose.position.y, feedback->current_pose.pose.position.z,
                    feedback->current_pose.pose.orientation.x, feedback->current_pose.pose.orientation.y, feedback->current_pose.pose.orientation.z,
                    feedback->current_pose.pose.orientation.w, feedback->current_pose.header.frame_id.c_str());
    }
}

void MoveBaseSkillNode::nav_result_cb(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(this->get_logger(), "[%s] Navigation goal reached", this->skill_name.c_str());
        this->goal_reached = true;
    }
    else if (result.code == rclcpp_action::ResultCode::ABORTED)
    {
        RCLCPP_WARN(this->get_logger(), "[%s] Navigation goal aborted", this->skill_name.c_str());
    }
    else if (result.code == rclcpp_action::ResultCode::CANCELED)
    {
        RCLCPP_WARN(this->get_logger(), "[%s] Navigation goal cancelled", this->skill_name.c_str());
    }

    this->navigation_complete = true;
}
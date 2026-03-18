#include "moveit_controller/cartesian_controller.hpp"

MoveitCartesianController::MoveitCartesianController(const rclcpp::NodeOptions &node_options)
    : rclcpp::Node("moveit_cartesian_controller", node_options)
{
    this->declare_parameter("max_moveit_initialisation_attempts", 10);
    this->declare_parameter("move_group_name", "arm_torso");
    this->declare_parameter("planner_id", "RRTstarkConfigDefault");
    this->declare_parameter("number_of_planning_attempts", 1);
    this->declare_parameter("max_velocity_scaling_factor", 0.1);
    this->declare_parameter("max_acceleration_scaling_factor", 0.1);
    this->declare_parameter("request_topic", "get_trajectory");
    this->declare_parameter("result_topic", "trajectory_execution_result");

    this->get_parameter("max_moveit_initialisation_attempts", this->max_moveit_initialisation_attempts);
    this->get_parameter("move_group_name", this->move_group_name);
    this->get_parameter("planner_id", this->planner_id);
    this->get_parameter("number_of_planning_attempts", this->number_of_planning_attempts);
    this->get_parameter("max_velocity_scaling_factor", this->max_velocity_scaling_factor);
    this->get_parameter("max_acceleration_scaling_factor", this->max_acceleration_scaling_factor);
    this->get_parameter("request_topic", this->request_topic);
    this->get_parameter("result_topic", this->result_topic);

    this->new_request_received = false;
    this->trajectory_execution_result_pub = this->create_publisher<std_msgs::msg::Bool>(this->result_topic, 10);
    this->trajectory_request_sub = this->create_subscription<geometry_msgs::msg::PoseArray>(this->request_topic,
                                                                                            10,
                                                                                            std::bind(&MoveitCartesianController::trajectory_request_cb, this, _1));
}

void MoveitCartesianController::initialise()
{
    RCLCPP_INFO(this->get_logger(), "Initialising MoveIt interface with group %s", this->move_group_name.c_str());

    bool moveit_initialised = false;
    int current_initialisation_attempt = 0;
    while (!moveit_initialised && (current_initialisation_attempt < this->max_moveit_initialisation_attempts))
    {
        RCLCPP_INFO(this->get_logger(), "Trying to initialise MoveGroupInterface");
        try
        {
            this->move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), this->move_group_name);
            moveit_initialised = true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "MoveGroupInterface failed to initialise: %s", e.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
            current_initialisation_attempt += 1;
        }
    }

    if (this->planner_id != "")
    {
        RCLCPP_INFO(this->get_logger(), "Setting planner to %s", this->planner_id.c_str());
        this->move_group->setPlannerId(this->planner_id);
    }

    RCLCPP_INFO(this->get_logger(), "Setting number of planning attempts to %d", this->number_of_planning_attempts);
    this->move_group->setNumPlanningAttempts(this->number_of_planning_attempts);

    RCLCPP_INFO(this->get_logger(), "Setting velocity scaling factor to %f", this->max_velocity_scaling_factor);
    this->move_group->setMaxVelocityScalingFactor(this->max_velocity_scaling_factor);

    RCLCPP_INFO(this->get_logger(), "Setting acceleration scaling factor to %f", this->max_acceleration_scaling_factor);
    this->move_group->setMaxAccelerationScalingFactor(this->max_acceleration_scaling_factor);

    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", this->move_group->getPlanningFrame().c_str());
    
    std::string ee_frame = this->move_group->getEndEffectorLink();
    RCLCPP_INFO(this->get_logger(), "End-effector frame: %s", ee_frame.c_str());

    geometry_msgs::msg::PoseStamped current_pose = this->move_group->getCurrentPose();
    RCLCPP_INFO(this->get_logger(),
                "Current EE Pose:\n frame_id=%s\n x=%f\n y=%f\n z=%f\n qx=%f\n qy=%f\n qz=%f\n qw=%f",
                current_pose.header.frame_id.c_str(),
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z,
                current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                current_pose.pose.orientation.z, current_pose.pose.orientation.w);
    RCLCPP_INFO(this->get_logger(), "MoveIt interface initialised with group %s", this->move_group_name.c_str());
}

void MoveitCartesianController::execute_trajectory()
{
    geometry_msgs::msg::PoseStamped current_pose = this->move_group->getCurrentPose();
    RCLCPP_INFO(this->get_logger(), "Executing trajectory");

    for (unsigned int i = 0; i < this->waypoints.size(); i++)
    {
        RCLCPP_INFO(this->get_logger(), "Planning path to pose %u:\n x=%f\n y=%f\n z=%f\n qx=%f\n qy=%f\n qz=%f\n qw=%f",
                    i, this->waypoints[i].position.x, this->waypoints[i].position.y, this->waypoints[i].position.z,
                    this->waypoints[i].orientation.x, this->waypoints[i].orientation.y, this->waypoints[i].orientation.z, this->waypoints[i].orientation.w);
    
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        this->move_group->setPoseTarget(this->waypoints[i]);
        this->move_group->setPlanningTime(3.0);
        auto planning_status = this->move_group->plan(plan);
        RCLCPP_INFO(this->get_logger(), "Planning status: %d", planning_status.val);

        robot_trajectory::RobotTrajectory trajectory(this->move_group->getRobotModel(), this->move_group->getName());
        trajectory.setRobotTrajectoryMsg(*this->move_group->getCurrentState(), plan.trajectory_);
        trajectory_processing::TimeOptimalTrajectoryGeneration trajectory_generation;
        trajectory_generation.computeTimeStamps(trajectory, this->max_velocity_scaling_factor, this->max_acceleration_scaling_factor);
        trajectory.getRobotTrajectoryMsg(plan.trajectory_);

        RCLCPP_INFO(this->get_logger(), "Executing plan...");
        auto success = this->move_group->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Plan execution successful");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Plan execution unsuccessful");
        }
    }

    execution_result_msg.data = true;
    this->trajectory_execution_result_pub->publish(this->execution_result_msg);
    this->new_request_received = false;
    RCLCPP_INFO(this->get_logger(), "Trajectory execution complete");
}

void MoveitCartesianController::trajectory_request_cb(const geometry_msgs::msg::PoseArray &request_msg)
{
    if (this->new_request_received)
    {
        RCLCPP_WARN(this->get_logger(), "Another trajectory is currently being executed; overwriting trajectory goals.");
    }

    RCLCPP_INFO(this->get_logger(), "Received trajectory request with %lu poses", request_msg.poses.size());
    if (request_msg.poses.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Received empty pose list; ignoring request");
        return;
    }

    this->waypoints = request_msg.poses;
    this->new_request_received = true;
    this->execution_result_msg.data = false;
}

bool MoveitCartesianController::execution_request_received()
{
    return this->new_request_received;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options = rclcpp::NodeOptions().allow_undeclared_parameters(true);
    auto node = std::make_shared<MoveitCartesianController>(options);

    // Moveit parameter initialisation based on https://github.com/bit-bots/bitbots_motion/blob/00cd3eea5dd906bff8ae615eeab734c556dd40ff/bitbots_moveit_bindings/src/bitbots_moveit_bindings.cpp#L24-L40
    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");
    while (!parameters_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for parameters client service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(node->get_logger(), "Parameter service not available, waiting...");
    }
    rcl_interfaces::msg::ListParametersResult parameter_list = parameters_client->list_parameters({"robot_description_kinematics"}, 10);
    auto parameters = parameters_client->get_parameters(parameter_list.names);
    node->set_parameters(parameters);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    node->initialise();
    while (rclcpp::ok())
    {
        try
        {
            if (node->execution_request_received())
            {
                RCLCPP_INFO(node->get_logger(), "Received new execution request; executing...");
                node->execute_trajectory();
            }
        }
        catch (const rclcpp::exceptions::RCLError& e)
        {
            RCLCPP_ERROR(node->get_logger(), "Node unexpectedly failed with %s", e.what());
        }
    }

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}

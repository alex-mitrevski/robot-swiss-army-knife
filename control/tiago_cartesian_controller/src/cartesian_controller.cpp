#include "tiago_cartesian_controller/cartesian_controller.hpp"

CartesianController::CartesianController(const rclcpp::NodeOptions &node_options)
    : rclcpp::Node("cartesian_controller", node_options)
{
    this->declare_parameter("goal_topic", "/cartesian_pose_goal");
    this->declare_parameter("arm_joint_controller_topic", "/arm_vel_controller/joint_trajectory");
    this->declare_parameter("torso_joint_controller_topic", "/torso_controller/joint_trajectory");
    this->declare_parameter("joint_states_topic", "/joint_states");
    this->declare_parameter("base_link_frame_name", "base_footprint");
    this->declare_parameter("end_effector_frame_name", "arm_tool_link");
    this->declare_parameter("joint_names", std::vector<std::string>({"torso_lift_joint", "arm_1_joint", "arm_2_joint", "arm_3_joint",
                                                                     "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"}));
    this->declare_parameter("torso_joint_names", std::vector<std::string>({"torso_lift_joint"}));
    this->declare_parameter("arm_joint_names", std::vector<std::string>({"arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint",
                                                                         "arm_5_joint", "arm_6_joint", "arm_7_joint"}));
    this->declare_parameter("min_arm_joint_vel_limits", std::vector<double>({-0.2, -0.2, -0.2, -0.2, -0.2, -0.2, -0.2}));
    this->declare_parameter("max_arm_joint_vel_limits", std::vector<double>({0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2}));
    this->declare_parameter("goal_threshold_m", 0.1);
    this->declare_parameter("goal_threshold_rad", 0.1);
    this->declare_parameter("debug_mode", false);

    this->get_parameter("goal_topic", this->goal_topic);
    this->get_parameter("arm_joint_controller_topic", this->arm_joint_controller_topic);
    this->get_parameter("torso_joint_controller_topic", this->torso_joint_controller_topic);
    this->get_parameter("joint_states_topic", this->joint_states_topic);
    this->get_parameter("base_link_frame_name", this->base_link_frame_name);
    this->get_parameter("end_effector_frame_name", this->end_effector_frame_name);
    this->get_parameter("joint_names", this->joint_names);
    this->get_parameter("arm_joint_names", this->arm_joint_names);
    this->get_parameter("torso_joint_names", this->torso_joint_names);
    this->get_parameter("min_arm_joint_vel_limits", this->min_arm_joint_vel_limits);
    this->get_parameter("max_arm_joint_vel_limits", this->max_arm_joint_vel_limits);
    this->get_parameter("goal_threshold_m", this->goal_threshold_m);
    this->get_parameter("goal_threshold_rad", this->goal_threshold_rad);
    this->get_parameter("debug_mode", this->debug_mode);

    this->kinematics_interface = std::make_shared<kinematics_interface_kdl::KinematicsInterfaceKDL>();

    this->tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer);

    this->arm_joint_trajectory_goal.joint_names = this->arm_joint_names;
    this->torso_joint_trajectory_goal.joint_names = this->torso_joint_names;

    this->arm_trajectory_goal_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(this->arm_joint_controller_topic, 10);
    this->torso_trajectory_goal_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(this->torso_joint_controller_topic, 10);
    this->joint_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(this->joint_states_topic,
                                                                                    10,
                                                                                    std::bind(&CartesianController::joint_state_cb, this, _1));

    RCLCPP_INFO(this->get_logger(), "Subscribing to geometry_msgs/msg/PoseStamped goals on topic %s", this->goal_topic.c_str());
    this->cartesian_goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(this->goal_topic,
                                                                                          10,
                                                                                          std::bind(&CartesianController::cartesian_goal_cb, this, _1));

    this->names_to_idx_map_initialised = false;
    this->executing_goal = false;
}

bool CartesianController::init_kinematics_interface()
{
    try
    {
        RCLCPP_INFO(this->get_logger(), "Initialising kinematics interface...");
        this->kinematics_interface->initialize(this->get_node_parameters_interface(),
                                               this->end_effector_frame_name);
        RCLCPP_INFO(this->get_logger(), "Kinematics interface initialised");
        return true;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Kinematics interface could not be initialised: %s", e.what());
        return false;
    }
}

void CartesianController::cartesian_goal_cb(const geometry_msgs::msg::PoseStamped &pose_stamped_msg)
{
    if (this->executing_goal)
    {
        RCLCPP_WARN(this->get_logger(), "Already pursuing another goal; overwriting current goal");
    }

    this->current_cartesian_goal.x = pose_stamped_msg.pose.position.x;
    this->current_cartesian_goal.y = pose_stamped_msg.pose.position.y;
    this->current_cartesian_goal.z = pose_stamped_msg.pose.position.z;

    tf2::Quaternion goal_quaternion(pose_stamped_msg.pose.orientation.x,
                                   pose_stamped_msg.pose.orientation.y,
                                   pose_stamped_msg.pose.orientation.z,
                                   pose_stamped_msg.pose.orientation.w);
    tf2::Matrix3x3 orientation_matrix(goal_quaternion);
    orientation_matrix.getRPY(this->current_cartesian_goal.roll,
                              this->current_cartesian_goal.pitch,
                              this->current_cartesian_goal.yaw);

    this->new_goal_received = true;
    RCLCPP_INFO(this->get_logger(),
                "Received new goal: %f, %f, %f, %f, %f, %f",
                this->current_cartesian_goal.x, this->current_cartesian_goal.y, this->current_cartesian_goal.z,
                this->current_cartesian_goal.roll, this->current_cartesian_goal.pitch, this->current_cartesian_goal.yaw);
}

void CartesianController::joint_state_cb(const sensor_msgs::msg::JointState &joint_state_msg)
{
    this->latest_joint_state_msg = joint_state_msg;

    if (!this->names_to_idx_map_initialised)
    {
        for (std::string name : this->arm_joint_trajectory_goal.joint_names)
        {
            for (unsigned int i=0; i<this->latest_joint_state_msg.name.size(); i++)
            {
                if (name == this->latest_joint_state_msg.name[i])
                {
                    RCLCPP_INFO(this->get_logger(), "Joint %s is in position %u", name.c_str(), i);
                    this->joint_name_to_idx_map[name] = i;
                    break;
                }
            }
        }

        for (std::string name : this->torso_joint_trajectory_goal.joint_names)
        {
            for (unsigned int i=0; i<this->latest_joint_state_msg.name.size(); i++)
            {
                if (name == this->latest_joint_state_msg.name[i])
                {
                    RCLCPP_INFO(this->get_logger(), "Joint %s is in position %u", name.c_str(), i);
                    this->joint_name_to_idx_map[name] = i;
                    break;
                }
            }
        }
        this->names_to_idx_map_initialised = true;
    }
}

void CartesianController::get_end_effector_pose()
{
    try
    {
        geometry_msgs::msg::TransformStamped tf_msg = this->tf_buffer->lookupTransform(this->base_link_frame_name,
                                                                                       this->end_effector_frame_name,
                                                                                       tf2::TimePointZero);
        this->latest_end_effector_pose.x = tf_msg.transform.translation.x;
        this->latest_end_effector_pose.y = tf_msg.transform.translation.y;
        this->latest_end_effector_pose.z = tf_msg.transform.translation.z;

        tf2::Quaternion quaternion(tf_msg.transform.rotation.x,
                                  tf_msg.transform.rotation.y,
                                  tf_msg.transform.rotation.z,
                                  tf_msg.transform.rotation.w);
        tf2::Matrix3x3 orientation_matrix(quaternion);
        orientation_matrix.getRPY(this->latest_end_effector_pose.roll,
                                  this->latest_end_effector_pose.pitch,
                                  this->latest_end_effector_pose.yaw);
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Could not transform %s to %s: %s",
                    this->end_effector_frame_name.c_str(),
                    this->base_link_frame_name.c_str(),
                    ex.what());
    }
}

void CartesianController::stop_motion()
{
    Eigen::Matrix<double, 8, 1> current_joint_positions = Eigen::Matrix<double, 8, 1>::Zero();;
    for (unsigned int i=0; i<8; i++)
    {
        current_joint_positions(i, 0) = this->latest_joint_state_msg.position[this->joint_name_to_idx_map[this->joint_names[i]]];
    }

    trajectory_msgs::msg::JointTrajectoryPoint torso_traj_point;
    torso_traj_point.positions = {current_joint_positions (0, 0)};
    torso_traj_point.time_from_start.sec = 1;
    this->torso_joint_trajectory_goal.points = {torso_traj_point};

    trajectory_msgs::msg::JointTrajectoryPoint arm_traj_point;
    arm_traj_point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    arm_traj_point.time_from_start.sec = 1;
    this->arm_joint_trajectory_goal.points = {arm_traj_point};

    this->torso_trajectory_goal_pub->publish(this->torso_joint_trajectory_goal);
    this->arm_trajectory_goal_pub->publish(this->arm_joint_trajectory_goal);
}

void CartesianController::execute_goal()
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    this->executing_goal = true;

    bool goal_reached = false;
    double position_norm = 1e10;
    Eigen::Matrix<double, 8, 1> current_joint_positions = Eigen::Matrix<double, 8, 1>::Zero();;
    Eigen::Matrix<double, 6, 1> pose_delta = Eigen::Matrix<double, 6, 1>::Zero();;
    Eigen::Matrix<double, Eigen::Dynamic, 1> joint_position_deltas = Eigen::Matrix<double, 8, 1>::Zero();;

    while (!goal_reached)
    {
        for (unsigned int i=0; i<8; i++)
        {
            current_joint_positions(i, 0) = this->latest_joint_state_msg.position[this->joint_name_to_idx_map[this->joint_names[i]]];
        }

        pose_delta(0, 0) = this->current_cartesian_goal.x - this->latest_end_effector_pose.x;
        pose_delta(1, 0) = this->current_cartesian_goal.y - this->latest_end_effector_pose.y;
        pose_delta(2, 0) = this->current_cartesian_goal.z - this->latest_end_effector_pose.z;
        pose_delta(3, 0) = this->current_cartesian_goal.roll - this->latest_end_effector_pose.roll;
        pose_delta(4, 0) = this->current_cartesian_goal.pitch - this->latest_end_effector_pose.pitch;
        pose_delta(5, 0) = this->current_cartesian_goal.yaw - this->latest_end_effector_pose.yaw;

        this->kinematics_interface->convert_cartesian_deltas_to_joint_deltas(current_joint_positions,
                                                                             pose_delta,
                                                                             this->end_effector_frame_name,
                                                                             joint_position_deltas);

        if (this->debug_mode)
        {
            RCLCPP_INFO(this->get_logger(),
                        "Received the following joint position deltas: %f, %f, %f, %f, %f, %f, %f, %f",
                        joint_position_deltas(0, 0), joint_position_deltas(1, 0), joint_position_deltas(2, 0), joint_position_deltas(3, 0),
                        joint_position_deltas(4, 0), joint_position_deltas(5, 0), joint_position_deltas(6, 0), joint_position_deltas(7, 0));
        }

        trajectory_msgs::msg::JointTrajectoryPoint torso_traj_point;
        torso_traj_point.positions = {current_joint_positions (0, 0) + joint_position_deltas(0, 0)};
        torso_traj_point.time_from_start.sec = 5;
        this->torso_joint_trajectory_goal.points = {torso_traj_point};

        trajectory_msgs::msg::JointTrajectoryPoint arm_traj_point;

        // Note that the first entry in 'joint_positions' corresponds to the
        // position delta of the torso, which is why the indices are shifted
        arm_traj_point.velocities = {std::clamp(joint_position_deltas(1, 0), this->min_arm_joint_vel_limits[0], this->max_arm_joint_vel_limits[0]),
                                     std::clamp(joint_position_deltas(2, 0), this->min_arm_joint_vel_limits[1], this->max_arm_joint_vel_limits[1]),
                                     std::clamp(joint_position_deltas(3, 0), this->min_arm_joint_vel_limits[2], this->max_arm_joint_vel_limits[2]),
                                     std::clamp(joint_position_deltas(4, 0), this->min_arm_joint_vel_limits[3], this->max_arm_joint_vel_limits[3]),
                                     std::clamp(joint_position_deltas(5, 0), this->min_arm_joint_vel_limits[4], this->max_arm_joint_vel_limits[4]),
                                     std::clamp(joint_position_deltas(6, 0), this->min_arm_joint_vel_limits[5], this->max_arm_joint_vel_limits[5]),
                                     std::clamp(joint_position_deltas(7, 0), this->min_arm_joint_vel_limits[6], this->max_arm_joint_vel_limits[6])};
        arm_traj_point.time_from_start.sec = 1;
        this->arm_joint_trajectory_goal.points = {arm_traj_point};

        this->torso_trajectory_goal_pub->publish(this->torso_joint_trajectory_goal);
        this->arm_trajectory_goal_pub->publish(this->arm_joint_trajectory_goal);
        rclcpp::sleep_for(std::chrono::milliseconds(200));

        // we check if we have now reached the goal
        this->get_end_effector_pose();
        position_norm = CartesianPose::get_position_norm(this->current_cartesian_goal, this->latest_end_effector_pose);
        goal_reached = position_norm < this->goal_threshold_m;

        if (this->debug_mode)
        {
            RCLCPP_INFO(this->get_logger(), "Current position norm: %f --- Norm threshold: %f", position_norm, this->goal_threshold_m);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Goal reached");
    this->executing_goal = false;
    this->new_goal_received = false;
    this->stop_motion();
}

void spin(rclcpp::Node::SharedPtr node)
{
    rclcpp::executors::SingleThreadedExecutor exec;
    rclcpp::WallRate loop_rate(20ms);
    while(rclcpp::ok())
    {
        exec.spin_node_some(node);
        loop_rate.sleep();
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options = rclcpp::NodeOptions().allow_undeclared_parameters(true);
    auto cartesian_controller_node = std::make_shared<CartesianController>(options);

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(cartesian_controller_node, "/robot_state_publisher");
    while (!parameters_client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(cartesian_controller_node->get_logger(), "Interrupted while waiting for parameters client service. Exiting.");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(cartesian_controller_node->get_logger(), "Parameter service not available, waiting...");
    }
    rcl_interfaces::msg::ListParametersResult parameter_list = parameters_client->list_parameters({"robot_description"}, 10);
    auto parameters = parameters_client->get_parameters(parameter_list.names);
    cartesian_controller_node->set_parameters(parameters);

    rclcpp::WallRate loop_rate(500ms);
    std::thread spinner(spin, cartesian_controller_node);

    bool kinematics_interface_initialised = cartesian_controller_node->init_kinematics_interface();
    if (!kinematics_interface_initialised)
    {
        RCLCPP_ERROR(cartesian_controller_node->get_logger(), "Kinematics interface could not be initialised. Exiting.");
        rclcpp::shutdown();
    }

    while (rclcpp::ok())
    {
        try
        {
            if (cartesian_controller_node->new_goal_received)
            { 
                cartesian_controller_node->execute_goal();
            }
        }
        catch (const rclcpp::exceptions::RCLError& e)
        {
            RCLCPP_ERROR(cartesian_controller_node->get_logger(), "Node unexpectedly failed with %s", e.what());
        }
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
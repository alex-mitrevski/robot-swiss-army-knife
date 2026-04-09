#include "point_cloud_utils/pose_calculator.hpp"

PoseCalculator::PoseCalculator()
    : rclcpp::Node("pose_calculator")
{
    this->declare_parameter("pose_calculation_srv_name", "calculate_pose_from_cloud");
    this->get_parameter("pose_calculation_srv_name", this->pose_calculation_srv_name);

    this->pose_calculation_server = this->create_service<robot_swiss_knife_msgs::srv::CalculatePoseFromCloud>(this->pose_calculation_srv_name,
                                                                                                              std::bind(&PoseCalculator::calculate_pose, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Pose calculation service exposed as %s; waiting for requests", this->pose_calculation_srv_name.c_str());
}

void PoseCalculator::calculate_pose(const std::shared_ptr<robot_swiss_knife_msgs::srv::CalculatePoseFromCloud::Request> request,
                                    std::shared_ptr<robot_swiss_knife_msgs::srv::CalculatePoseFromCloud::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received a pose calculation request");
    if (request->point_cloud.data.size() == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Empty point cloud received; ignoring request");
        response->calculation_successful = false;
    }
    else
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(request->point_cloud, *pcl_cloud);

        pcl::PCA<pcl::PointXYZRGB> pca_interface;
        pca_interface.setInputCloud(pcl_cloud);
        Eigen::Vector4f cloud_centroid = pca_interface.getMean();
        Eigen::Matrix3f basis_vectors = pca_interface.getEigenVectors();
        basis_vectors.col(2) = basis_vectors.col(0).cross(basis_vectors.col(1));

        response->pose.header.frame_id = request->point_cloud.header.frame_id;
        response->pose.header.stamp = this->get_clock()->now();
        response->pose.pose.position.x = cloud_centroid(0);
        response->pose.pose.position.y = cloud_centroid(1);
        response->pose.pose.position.z = cloud_centroid(2);

        tf2::Matrix3x3 rotation_matrix_tf(basis_vectors(0, 0), basis_vectors(0, 1), basis_vectors(0, 2),
                                        basis_vectors(1, 0), basis_vectors(1, 1), basis_vectors(1, 2),
                                        basis_vectors(2, 0), basis_vectors(2, 1), basis_vectors(2, 2));
        tf2::Quaternion quaternion;
        rotation_matrix_tf.getRotation(quaternion);

        response->pose.pose.orientation.x = quaternion[0];
        response->pose.pose.orientation.y = quaternion[1];
        response->pose.pose.orientation.z = quaternion[2];
        response->pose.pose.orientation.w = quaternion[3];

        response->calculation_successful = true;
        RCLCPP_INFO(this->get_logger(), "Pose calculation completed");
    }
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
    auto pose_calculation_node = std::make_shared<PoseCalculator>();
    rclcpp::WallRate loop_rate(500ms);
    std::thread spinner(spin, pose_calculation_node);

    while (rclcpp::ok())
    {
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
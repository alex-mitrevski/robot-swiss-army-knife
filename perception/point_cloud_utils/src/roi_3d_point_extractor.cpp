#include "point_cloud_utils/roi_3d_point_extractor.hpp"

ROI3DPointExtractor::ROI3DPointExtractor()
    : rclcpp::Node("roi_3d_point_extractor")
{
    this->declare_parameter("extraction_srv_name", "extract_3d_rois");
    this->get_parameter("extraction_srv_name", this->extraction_srv_name);

    this->extraction_server = this->create_service<robot_swiss_knife_msgs::srv::ExtractROI3DPoints>(this->extraction_srv_name,
                                                                                                    std::bind(&ROI3DPointExtractor::extract_rois, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Extraction service exposed as %s; waiting for requests", this->extraction_srv_name.c_str());
}

void ROI3DPointExtractor::extract_rois(const std::shared_ptr<robot_swiss_knife_msgs::srv::ExtractROI3DPoints::Request> request,
                                       std::shared_ptr<robot_swiss_knife_msgs::srv::ExtractROI3DPoints::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Received an ROI extraction request for %lu object masks", request->object_masks.size());
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
    pcl::fromROSMsg(request->point_cloud, pcl_cloud);

    unsigned int mask_count = 1;
    for (const sensor_msgs::msg::Image& mask : request->object_masks)
    {
        RCLCPP_INFO(this->get_logger(), "Extracting region of object %u", mask_count);

        pcl::PointCloud<pcl::PointXYZRGB> object_cloud_pcl;
        std::vector<unsigned int> nonzero_mask_indices = this->get_nonzero_indices(mask);
        RCLCPP_INFO(this->get_logger(), "Found %lu points for object %u", nonzero_mask_indices.size(), mask_count);

        unsigned int valid_point_count = 0;
        for (unsigned int idx : nonzero_mask_indices)
        {
            if (pcl::isFinite(pcl_cloud[idx]))
            {
                valid_point_count += 1;
                object_cloud_pcl.push_back(pcl_cloud[idx]);
            }
        }
        object_cloud_pcl.width = valid_point_count;
        object_cloud_pcl.height = 1;
        object_cloud_pcl.is_dense = true;

        sensor_msgs::msg::PointCloud2::SharedPtr object_cloud(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(object_cloud_pcl, *object_cloud);
        object_cloud->header.frame_id = request->point_cloud.header.frame_id;
        object_cloud->header.stamp = this->get_clock()->now();
        response->object_clouds.push_back(*object_cloud);

        mask_count += 1;
    }
    RCLCPP_INFO(this->get_logger(), "Region extraction completed");
}

const std::vector<unsigned int> ROI3DPointExtractor::get_nonzero_indices(const sensor_msgs::msg::Image &image_msg) const
{
    RCLCPP_INFO(this->get_logger(), "Extracting non-zero indices for mask");
    std::vector<unsigned int> nonzero_indices;
    unsigned int item_counter = 0;
    for (auto item : image_msg.data)
    {
        if (item > 0)
        {
            nonzero_indices.push_back(item_counter);
        }
	item_counter += 1;
    }
    RCLCPP_INFO(this->get_logger(), "Mask processing complete");
    return nonzero_indices;
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
    auto roi_extractor_node = std::make_shared<ROI3DPointExtractor>();
    rclcpp::WallRate loop_rate(500ms);
    std::thread spinner(spin, roi_extractor_node);

    while (rclcpp::ok())
    {
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

#ifndef ROI_3D_POINT_EXTRACTOR_HPP
#define ROI_3D_POINT_EXTRACTOR_HPP

#include <thread>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/common.h>
#include <pcl/common/point_tests.h>
#include <pcl_conversions/pcl_conversions.h>

#include <robot_swiss_knife_msgs/srv/extract_roi3_d_points.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * A component for extracting 3D points corresponding to a given
 * region of interest in an image. The component assumes that the
 * used point cloud is organised.
 * 
 * The component uses some ideas from the following repository:
 * https://github.com/adrian-soch/pcl_tutorial
 *
 * @author Alex Mitrevski
 * @contact alemitr@chalmers.se
 */
class ROI3DPointExtractor : public rclcpp::Node
{
public:
    ROI3DPointExtractor();
private:
    /**
     * Service callback that extracts point cloud points that correspond to objects
     * represented by segmentation masks (each mask representing a separate object).
     *
     * @param request Service request containing an input point cloud and segmentation masks of objects
     * @param response Service response containing point clouds corresponding to each of the objects
     */
    void extract_rois(const std::shared_ptr<robot_swiss_knife_msgs::srv::ExtractROI3DPoints::Request> request,
                      std::shared_ptr<robot_swiss_knife_msgs::srv::ExtractROI3DPoints::Response> response);

    /**
     * Extracts the indices of all non-zero pixels in the input image.
     *
     * @param image_msg Image interpreted as a segmentation mask (with values larger
     *                  than zero corresponding to pixels belonging to the mask)
     */
    const std::vector<unsigned int> get_nonzero_indices(const sensor_msgs::msg::Image &image_msg) const;

    rclcpp::Service<robot_swiss_knife_msgs::srv::ExtractROI3DPoints>::SharedPtr extraction_server;
    std::string extraction_srv_name;
};

#endif
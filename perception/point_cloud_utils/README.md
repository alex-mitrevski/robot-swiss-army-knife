# point_cloud_utils

A package containing various utility components that perform operations on point clouds.

## Dependencies

* `pcl_ros`, `pcl_conversions`, `tf2_ros`, `tf2_eigen`
* [robot_swiss_knife_msgs](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/robot_swiss_knife_msgs)

## Usage instructions

### ROI point extraction

A component for extracting subsets of a point cloud that corresponds to regions of interests specified by segmentation masks can be launched as follows:
```
ros2 launch point_cloud_utils roi_extractor.launch.py
```
The component exposes a service for ROI extraction, whose interface is of type [robot_swiss_knife_msgs/srv/ExtractROI3DPoints](https://github.com/alex-mitrevski/robot-swiss-army-knife/blob/main/robot_swiss_knife_msgs/srv/ExtractROI3DPoints.srv). The service name (`/extract_3d_rois` by default) can be specified in the launch file.

Note that the component makes an assumption that the point cloud is organised (as we need the pixel-to-point mapping to be known).

### Pose calculation

A component for calculating a 3D pose of an object specified by a point cloud can be launched as follows:
```
ros2 launch point_cloud_utils pose_calculator.launch.py
```
This component also exposes a service for pose calculation, which is of type [robot_swiss_knife_msgs/srv/CalculatePoseFromCloud](https://github.com/alex-mitrevski/robot-swiss-army-knife/blob/main/robot_swiss_knife_msgs/srv/CalculatePoseFromCloud.srv). The service name (`/calculate_pose_from_cloud` by default) can be specified in the launch file.

Internally, the component uses PCA for pose calculation.
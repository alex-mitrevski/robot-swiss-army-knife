# behaviour_library

A library of behaviours and behaviour trees for various robot tasks, based on BehaviorTree.CPP and BehaviorTree.ROS2. The behaviours are built as plugins that can then be integrated into trees.

## Dependencies

* [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) (can be installed with apt: `apt install ros-humble-behaviortree-cpp*`)
* [BehaviorTree.ROS2](https://github.com/BehaviorTree/BehaviorTree.ROS2/tree/humble) (should be cloned into the ROS 2 workspace)
* [robot_swiss_knife_msgs](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/robot_swiss_knife_msgs)
* [move_base_skill](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/skills/move_base_skill)

## Usage instructions

After building, the tree executor can be launched as follows:
```
ros2 launch behaviour_library tree_executor.launch.xml
```
By default, this launches an example tree executor, which is currently the only executor available (using an [example configuration file](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/behaviour_library/config/example-config.yaml) and an [example tree](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/behaviour_library/behaviour_trees/example-tree.xml)). The launch file entries should be changed accordingly if another executor and / or configuration file should be used.

## Short descriptions of available behaviours

### Publisher / subscriber behaviours

* `ImageSubBehaviour`: Subscribes to `sensor_msgs/msg/Image` messages. Saves a received image message on the `latest_image` output port.
* `PointCloudSubBehaviour`: Subscribes to `sensor_msgs/msg/PointCloud2` messages. Saves a received image message on the `latest_point_cloud` output port.
* `PosePubBehaviour`: Publishes `geometry_msgs/msg/PoseStamped` messages. Reads the pose to be published from the `object_pose` input port.

### Service interface behaviours

* `CheckVisibilityBehaviour`: Acts as a client of the [object visibility checking component](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/perception/object_visibility_checker). Thus, expects an image (input port `latest_image`) and target object categories (input port `object_categories`) to be available on the blackboard. Saves the result on the `visible_objects` output port.
* `SegmentObjectsBehaviour`: Acts as a client of the [object segmentation component](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/perception/semantic_segmentation). Thus, expects an image (input port `latest_image`) and target object categories (input port `object_categories`) to be available on the blackboard. Saves the result on the `segmented_objects` output port.
* `ExtractROI3DPointsBehaviour`: Acts as a client of the [ROI cloud extraction component](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/perception/point_cloud_utils). Thus, expects a point cloud (input port `latest_point_cloud`) and segmented objects as produced by the `SegmentObjectsBehaviour` (input port `segmented_objects`) to be available on the blackboard. Saves the result on the `object_clouds` output port.
* `CalculatePoseBehaviour`: Acts as a client of the [cloud-based pose calculation component](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/perception/point_cloud_utils). Expects object points clouds as produced by the `ExtractROI3DPointsBehaviour` (input port `object_clouds`) and the name of the object of interest (input port `object_of_interest`) to be available on the blackboard. Saves the result on the `calculated_pose` output port. If the name is not available in `object_clouds` directly, looks for an existing name that contains the passed name (e.g. if the value passed on `object_of_interest` is `cup`, but `object_clouds` has an object `cup_0`, it will calculate the pose of `cup_0`).

### Skill behaviours

* `MoveBaseBehaviour`: Acts as a client of the [move base skill](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/skills/move_base_skill). Expects a skill goal type (input port `goal_type`) and navigation goals (input port `goal_poses` for goals specified as a list of `geometry_msgs/msg/PoseStamped` messages). Does not have any output ports.
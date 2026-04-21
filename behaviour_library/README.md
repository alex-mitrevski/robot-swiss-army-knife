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

* `ImageSubBehaviour`: Subscribes to image messages. Saves a received image message on the `latest_image` output port.
* `CheckVisibilityBehaviour`: Acts as a client of the [object visibility checking component](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/perception/object_visibility_checker). Thus, expects an image (input port `latest_image`) and target object categories (input port `object_categories`) to be available on the blackboard. Saves the result on the `visible_objects` output port.
* `MoveBaseBehaviour`: Acts as a client of the [move base skill](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/skills/move_base_skill). Expects a skill goal type (input port `goal_type`) and navigation goals (input port `goal_poses` for goals specified as a list of `geometry_msgs/msg/PoseStamped` messages). Does not have any output ports.
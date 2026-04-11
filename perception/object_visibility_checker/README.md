# object_visibility_checker

A Python-based component for checking whether objects are visible in the currently visible scene.

## Dependencies

* [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
* [robot_swiss_knife_msgs](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/robot_swiss_knife_msgs)

## Usage instructions

The component has a node that exposes a service for checking whether objects of given categories are visible in the current scene. After building this package, the node can be started as follows:
```
ros2 launch object_visibility_checker visibility_checker.launch.py
```
The service name (`/check_objects_visible` by default) can be specified in the launch file.

The service interface used by the component is of type [robot_swiss_knife_msgs/srv/CheckObjectVisibility](https://github.com/alex-mitrevski/robot-swiss-army-knife/blob/main/robot_swiss_knife_msgs/srv/CheckObjectVisibility.srv).
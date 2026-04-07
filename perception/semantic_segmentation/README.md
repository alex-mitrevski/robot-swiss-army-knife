# semantic_segmentation

A Python-based component with utilities for semantic object segmentation.

## Dependencies

* [Ultralytics YOLO](https://github.com/ultralytics/ultralytics)
* [SAM 2](https://github.com/facebookresearch/sam2)
* [robot_swiss_knife_msgs](https://github.com/alex-mitrevski/robot-swiss-army-knife/tree/main/robot_swiss_knife_msgs)

## Usage instructions

The component includes a node that exposes a service for object segmentation. After building this package, the node can be started as follows:
```
ros2 launch semantic_segmentation segmentation.launch.py
```
The service name (`/segment_objects` by default) can be specified in the launch file.

The service interface used by the component is of type [robot_swiss_knife_msgs/srv/SegmentObjects](https://github.com/alex-mitrevski/robot-swiss-army-knife/blob/main/robot_swiss_knife_msgs/srv/SegmentObjects.srv). An example client that illustrates the usage by performing segmentation on images from a TIAGo robot can be found [here](https://github.com/alex-mitrevski/robot-swiss-army-knife/blob/main/perception/semantic_segmentation/test/test_segmentation_client_tiago.py) in the `test` directory.
# object-pose-estimator
Collection of commands to run 

docker stop rsak
docker rm rsak
docker exec -it rsak bash

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

cd /ros2_ws
colcon build --packages-select semantic_segmentation object_pose_estimator

ros2 run semantic_segmentation segment_objects

ros2 run object_pose_estimator pose_estimator \
  --ros-args \
  -r /xtion/rgb/image_raw:=/head_front_camera/rgb/image_raw \
  -r /xtion/depth_registered/points:=/head_front_camera/depth/rgb/points \
  -r /xtion/rgb/camera_info:=/head_front_camera/rgb/camera_info \
  -p object_category:="blue bottle"

docker run -it \
  --gpus all \
  --network host \
  --name rsak \
  -e DISPLAY=${DISPLAY} \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /Users/ninad/Documents/gripper videos/episodes \
  -v /Users/ninad/Documents/gripper videos/robot-swiss-army-kniferobot-swiss-army-knife \
  rsak-saved \
  bash

export CYCLONEDDS_URI=/ros2_ws/src/robot-swiss-army-knife/docker/tiago-cyclone-config.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=79
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

ros2 daemon stop
ros2 daemon start

FROM osrf/ros:humble-desktop

SHELL [ "/bin/bash", "-c" ]
RUN apt update && apt upgrade -y
RUN apt update && apt install -y iproute2 \
                                 iputils-ping \
                                 vim \
                                 ranger \
                                 python3-pip \
                                 libpcl-dev \
                                 ros-humble-camera-info-manager \
                                 ros-humble-controller-manager-msgs \
                                 ros-humble-joy-linux \
                                 ros-humble-gazebo-msgs \
                                 ros-humble-control-msgs \
                                 ros-humble-nav2-msgs \
                                 ros-humble-moveit \
                                 ros-humble-ros2-control-test-assets \
                                 ros-humble-rmw-cyclonedds-cpp \
                                 ros-humble-tf-transformations \
                                 ros-humble-pcl-ros \
                                 ros-humble-pcl-conversions \
                                 ros-humble-behaviortree-cpp*
RUN pip3 install ipython \
                 pandas \
                 networkx \
                 opencv-python \
                 cv_bridge \
                 huggingface_hub \
                 transformers \
                 accelerate \
                 ultralytics \
                 ollama \
                 torch \
                 torchvision \
                 scikit-learn

# we install an older version of numpy because
# opencv_bridge is incompatible with numpy 2+
RUN pip3 install numpy==1.26.4

##########################################################################
# uncomment the following lines if you want to include SAM 2 in the image
##########################################################################
# RUN mkdir /workspace
# RUN cd /workspace && git clone https://github.com/facebookresearch/sam2.git && cd sam2 && pip install -e .
# RUN cd /workspace/sam2/checkpoints && \./download_ckpts.sh

###########################################################################
# uncomment the following line if you want to include SpaceOm in the image
###########################################################################
# RUN hf download remyxai/SpaceOm
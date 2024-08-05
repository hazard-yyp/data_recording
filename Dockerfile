FROM pytorch/pytorch:2.1.1-cuda12.1-cudnn8-devel

ARG DEBIAN_FRONTEND=noninteractive

# Install ROS Noetic
# Setup keys and source list for ROS
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release && \
    sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install basic apt tools, ROS Noetic, cv_bridge
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    vim git wget zsh python3-rosdep build-essential cmake python3-catkin-tools python3-pip \
    doxygen graphviz ros-noetic-ros-base ros-noetic-cv-bridge && \
    # Initialize rosdep
    rosdep init && rosdep update

# Install python packages
RUN pip install --no-cache-dir rospkg opencv-python

WORKDIR /usr/src/app

COPY image_classifier_node.py ./
COPY model.pth ./
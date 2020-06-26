# Ubuntu 18.04 with nvidia-docker2 beta opengl support
FROM nvidia/opengl:1.0-glvnd-devel-ubuntu18.04
ENV DEBIAN_FRONTEND noninteractive
ENV TZ=America/New_York
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install Dependencies
RUN apt-get update \
  && apt-get install -y \
  apt-utils \
  build-essential \
  cmake \
  git \
  gnupg \
  lsb-release \
  libusb-dev \
  sudo \
  vim \
  && apt-get clean

# Install ROS Melodic
RUN sudo bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
  && sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN sudo apt-get update \
  && sudo apt-get install -y \
  ros-melodic-ros-base \
  ros-melodic-rviz \
  && sudo apt-get clean \
  && sudo rosdep init \
  && rosdep update

# Other Deps
RUN sudo apt-get update \
  && sudo apt-get install -y \
  apt-utils \
  autoconf \
  libcairo2-dev \
  libcanberra-gtk-module \
  libcanberra-gtk3-module \
  libceres1 \
  libceres-dev \
  liblua5.3-dev \
  libopencv-core-dev \
  libprotobuf-dev \
  libsuitesparse-dev \
  libturbojpeg0-dev \
  protobuf-compiler \
  python-catkin-tools \
  python-minimal \
  python-numpy \
  python-rosinstall \
  python-rosinstall-generator \
  python-wstool \
  python3-numpy \
  ros-melodic-camera-info-manager \
  ros-melodic-cv-bridge \
  ros-melodic-eigen-conversions \
  ros-melodic-image-geometry \
  ros-melodic-image-proc \
  ros-melodic-image-transport-plugins \
  ros-melodic-opencv-apps \
  ros-melodic-pcl-ros \
  ros-melodic-random-numbers \
  ros-melodic-static-transform-mux \
  ros-melodic-tf-conversions \
  xterm \
  && sudo apt-get clean

# Set Working Directory
WORKDIR /home/developer


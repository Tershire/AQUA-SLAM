# ROS2 Jazzy
FROM osrf/ros:jazzy-desktop

# set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# update package lists and install basic utilities
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    vim \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# install ROS 2 dependencies
RUN apt-get update && apt-get install -y \
    ros-jazzy-image-transport \
    ros-jazzy-cv-bridge \
    ros-jazzy-sensor-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-octomap-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-sensor-msgs \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-pcl-conversions \
    && rm -rf /var/lib/apt/lists/*

# install dependencies
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    libvtk9-dev \
    libsuitesparse-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    libepoxy-dev \
    libglew-dev \
    libgl1-mesa-dev \
    libxkbcommon-x11-0 \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# install source build libraries
WORKDIR /tmp

# Pangolin
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install

# Sophus
RUN git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && mkdir build && cd build && \
    cmake .. && make -j$(nproc) && make install

# fmt
RUN apt-get update && apt-get install -y libfmt-dev || \
    (git clone https://github.com/fmtlib/fmt.git && cd fmt && mkdir build && cd build && \
     cmake .. && make -j$(nproc) && make install)

# workspace
WORKDIR /root/ros2_ws

# setup ROS environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

CMD ["bash"]


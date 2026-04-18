# ROS2 Jazzy
FROM osrf/ros:jazzy-desktop

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

# install dependencies
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libopencv-dev \
    libsuitesparse-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    && rm -rf /var/lib/apt/lists/*

# workspace
WORKDIR /root/ros2_ws

# setup ROS environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

CMD ["bash"]


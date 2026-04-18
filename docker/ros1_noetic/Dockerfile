# ROS Noetic with OpenCV 3.4.16, PCL, Pangolin, Eigen3, fmt, and Boost
FROM ubuntu:20.04

# Set environment variables to avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic

# Update package lists and install basic utilities
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    python3-pip \
    python3-dev \
    python3-setuptools \
    python3-distutils \
    && rm -rf /var/lib/apt/lists/*


# Add ROS repository and setup ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Install ROS dependencies first
RUN apt update && apt install -y ros-noetic-desktop-full && apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update


# Install catkin-tools via pip
RUN pip3 install --upgrade pip setuptools && \
    pip3 install catkin-tools

# Install Eigen3
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Boost with log_setup and log components
RUN apt-get update && apt-get install -y \
    libboost-all-dev \
    && rm -rf /var/lib/apt/lists/*

# Install OpenCV dependencies
RUN apt-get update && apt-get install -y cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev \
    && rm -rf /var/lib/apt/lists/*

# Install OpenCV 3.4.16 with contrib modules from source
WORKDIR /tmp
RUN git clone --depth 1 --branch 3.4.16 https://github.com/opencv/opencv.git && \
    git clone --depth 1 --branch 3.4.16 https://github.com/opencv/opencv_contrib.git && \
    cd opencv && \
    mkdir -p build && \
    cd build && \
    cmake \
        -D CMAKE_BUILD_TYPE=Release \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D OPENCV_EXTRA_MODULES_PATH=/tmp/opencv_contrib/modules \
        -D WITH_TBB=ON \
        -D WITH_V4L=ON \
        -D WITH_OPENGL=ON \
        -D WITH_CUDA=OFF \
        .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd /tmp && \
    rm -rf opencv opencv_contrib

# Install PCL dependencies
RUN apt-get update && apt-get install -y \
    libflann-dev \
    libqhull-dev \
    libusb-1.0-0-dev \
    libopenni-dev \
    libopenni2-dev \
    && rm -rf /var/lib/apt/lists/*

# Install PCL from apt (compatible with Noetic)
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    pcl-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Pangolin from source
WORKDIR /tmp
RUN git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    git checkout bcf41ec8b2853eb132dbe5cbe65ca8eade31846e && \
    mkdir -p build && \
    cd build && \
    cmake \
        -D CMAKE_BUILD_TYPE=Release \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        .. && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && \
    rm -rf Pangolin

# Install fmt library from source (to get modern CMake targets)
WORKDIR /tmp
RUN git clone https://github.com/fmtlib/fmt.git && \
    cd fmt && \
    git checkout 9.1.0 && \
    mkdir -p build && \
    cd build && \
    cmake \
        -D CMAKE_BUILD_TYPE=Release \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D FMT_TEST=OFF \
        .. && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && \
    rm -rf fmt

# Install Sophus library from source
WORKDIR /tmp
RUN git clone https://github.com/strasdat/Sophus.git && \
    cd Sophus && \
    git checkout 1.22.10 && \
    mkdir -p build && \
    cd build && \
    cmake \
        -D CMAKE_BUILD_TYPE=Release \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D BUILD_TESTS=OFF \
        .. && \
    make -j$(nproc) && \
    make install && \
    cd /tmp && \
    rm -rf Sophus

# Additional ROS packages that might be useful
RUN apt-get update && apt-get install -y clang\
    ros-noetic-rviz \
    ros-noetic-tf \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-octomap \
    ros-noetic-octomap-msgs \
    ros-noetic-octomap-ros \
    ros-noetic-octomap-server \
    && rm -rf /var/lib/apt/lists/*

# Minimal desktop plumbing for running RViz/Qt apps in Docker (X11 + DBus + OpenGL)
RUN apt-get update && apt-get install -y \
    dbus-x11 \
    xauth \
    mesa-utils \
    libgl1 \
    libgl1-mesa-dri \
    libegl1 \
    libxkbcommon-x11-0 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --user gdown

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/SenseRoboticsLab/dvl_a50_ros_driver.git

# Provide a runtime dir expected by Qt/desktop components (rviz) when running in minimal containers.
# If you later run as non-root, you can override this at runtime.
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# Create XDG runtime dir early so GUI apps don't crash when it doesn't exist.
RUN mkdir -p ${XDG_RUNTIME_DIR} && chmod 700 ${XDG_RUNTIME_DIR}

# Setup ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Create a workspace
WORKDIR /root/catkin_ws

# Initialize the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin init"

# Set the working directory
WORKDIR /root/catkin_ws
ENV DISABLE_ROS1_EOL_WARNINGS=1
# Keep the container running
CMD ["/bin/bash"]

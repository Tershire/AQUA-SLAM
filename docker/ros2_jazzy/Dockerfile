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
    ros-jazzy-image-transport-plugins \
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

# waterlinked_a50_ros_driver: minimal DVL message stub (not on apt)
RUN mkdir -p /root/ros2_ws/src/waterlinked_a50_ros_driver/msg

RUN cat > /root/ros2_ws/src/waterlinked_a50_ros_driver/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>waterlinked_a50_ros_driver</name>
  <version>0.0.1</version>
  <description>Minimal message definitions for WaterLinked A50 DVL (ROS2 stub)</description>
  <maintainer email="stub@stub.com">stub</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export><build_type>ament_cmake</build_type></export>
</package>
EOF

RUN cat > /root/ros2_ws/src/waterlinked_a50_ros_driver/CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.10)
project(waterlinked_a50_ros_driver)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DVLBeam.msg"
  "msg/DVL.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
ament_package()
EOF

RUN cat > /root/ros2_ws/src/waterlinked_a50_ros_driver/msg/DVLBeam.msg << 'EOF'
int64 id
float64 velocity
float64 distance
float64 rssi
float64 nsd
bool valid
EOF

RUN cat > /root/ros2_ws/src/waterlinked_a50_ros_driver/msg/DVL.msg << 'EOF'
std_msgs/Header header
float64 time
geometry_msgs/Vector3 velocity
float64 fom
float64 altitude
waterlinked_a50_ros_driver/DVLBeam[] beams
bool velocity_valid
int64 status
string form
EOF

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-select waterlinked_a50_ros_driver

# setup ROS environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["bash"]


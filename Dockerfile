FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# 1. Install system, Gazebo Harmonic, MAVROS, OctoMap, and Python/C++ build dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    lsb-release \
    gnupg \
    git \
    wget \
    curl \
    tmux \
    build-essential \
    cmake \
    ninja-build \
    python3-dev \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-numpy \
    python3-scipy \
    python3-pynput \
    libeigen3-dev \
    libpcl-dev \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-mavros \
    ros-jazzy-mavros-extras \
    ros-jazzy-octomap \
    ros-jazzy-octomap-msgs \
    ros-jazzy-octomap-ros \
    ros-jazzy-octomap-server \
    ros-jazzy-octomap-rviz-plugins \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets required by MAVROS
RUN /opt/ros/jazzy/lib/mavros/install_geographiclib_datasets.sh

# 2. Clone PX4-Autopilot at the exact verified commit (Ubuntu 24.04 + Gazebo Harmonic)
#    Path /home/fishman/PX4-Autopilot matches launch/local_px4_file.launch.py & px4_launch.launch.py
RUN mkdir -p /home/fishman && cd /home/fishman && \
    git clone https://github.com/PX4/PX4-Autopilot.git && \
    cd /home/fishman/PX4-Autopilot && \
    git checkout e17d81af21 && \
    git submodule update --init --recursive && \
    bash /home/fishman/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools

# 3. Create ROS 2 workspace and clone Point-LIO (dfloreaa/point_lio_ros2)
RUN mkdir -p /home/fishman/ros2_ws/src && cd /home/fishman/ros2_ws/src && \
    git clone https://github.com/dfloreaa/point_lio_ros2.git && \
    cd point_lio_ros2 && git checkout a8e2d0d

# 4. Copy this package into the workspace
COPY . /home/fishman/ros2_ws/src/uav_control_mapping

# 5. Apply external_files patches (custom 4022_gz_x500_lidar_3d airframe/models + Point-LIO config)
#    and pre-compile PX4 SITL
RUN cp -r /home/fishman/ros2_ws/src/uav_control_mapping/external_files/ROMFS /home/fishman/PX4-Autopilot/ && \
    cp -r /home/fishman/ros2_ws/src/uav_control_mapping/external_files/Tools /home/fishman/PX4-Autopilot/ && \
    cp -r /home/fishman/ros2_ws/src/uav_control_mapping/external_files/point_lio_ros2/* /home/fishman/ros2_ws/src/point_lio_ros2/ && \
    cd /home/fishman/PX4-Autopilot && DONT_RUN=1 make px4_sitl_default

# 6. Build the ROS 2 workspace (Point-LIO + uav_control_mapping)
WORKDIR /home/fishman/ros2_ws
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install --packages-select point_lio uav_control_mapping --cmake-args -DCMAKE_BUILD_TYPE=Release"

# 7. Auto-source ROS 2 and workspace in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /home/fishman/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]

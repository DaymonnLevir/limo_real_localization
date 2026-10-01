FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

# Dependências de compilação + dependências usadas pelo
# Livox-SDK2, livox_ros_driver2 e FAST-LIO2.
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-dev \
    python3-colcon-common-extensions \
    libeigen3-dev \
    libpcl-dev \
    libapr1-dev \
    libboost-all-dev \
    ros-humble-ament-cmake-auto \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rclcpp \
    ros-humble-rclcpp-components \
    ros-humble-rcutils \
    ros-humble-rcl-interfaces \
    ros-humble-std-msgs \
    ros-humble-std-srvs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-common-interfaces \
    ros-humble-tf2 \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-rosbag2 \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# ============================================================
# Livox-SDK2
# ============================================================

COPY Livox-SDK2 /workspace/Livox-SDK2

RUN cd /workspace/Livox-SDK2 && \
    rm -rf build && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j2 && \
    make install && \
    ldconfig

# ============================================================
# ROS 2 workspace
# ============================================================

COPY ros2_ws /workspace/ros2_ws

RUN source /opt/ros/humble/setup.bash && \
    cd /workspace/ros2_ws && \
    colcon build \
        --symlink-install \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DROS_EDITION=ROS2 \
            -DDISTRO_ROS=humble

# Ambiente automático ao abrir um terminal
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/ros2_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /workspace/ros2_ws

CMD ["bash"]

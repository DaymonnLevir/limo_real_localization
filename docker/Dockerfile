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
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-builtin-interfaces \
    ros-humble-teleop-twist-keyboard \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-rosbag2 \
    ros-humble-rviz2 \
    ros-humble-v4l2-camera \
    ros-humble-camera-info-manager \
    ros-humble-camera-calibration \
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
# FAST-LIVO2 compile-time dependencies (the runtime camera path is disabled).
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-dev libfmt-dev ros-humble-sophus \
    ros-humble-cv-bridge ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*
# The ROS Sophus package exports a target, while this fork reads legacy include variables.
# Match the upstream Docker include path for Vikit and FAST-LIVO2.
ENV CPATH=/opt/ros/humble/include
# Pinned U-AMC Vikit dependency; no additional Livox driver or SDK.
RUN git clone https://github.com/U-AMC/rpg_vikit_rational_polynomial.git /workspace/ros2_ws/src/rpg_vikit && \
    cd /workspace/ros2_ws/src/rpg_vikit && \
    git checkout 6f213c7fe6be1aff1fc79c7bd6103e3e9bc4a02d && \
    rm -rf .git && \
    source /opt/ros/humble/setup.bash && \
    cmake -S vikit_common -B /tmp/vikit_common_build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/vikit_common_build --parallel 2 && \
    cmake --install /tmp/vikit_common_build && \
    mkdir -p /usr/local/lib/cmake/vikit_common && \
    printf '%s\n' \
      'set(vikit_common_INCLUDE_DIRS "/usr/local/include")' \
      'set(vikit_common_INCLUDE_DIR "/usr/local/include")' \
      'set(vikit_common_LIBRARIES "/usr/local/lib/libvikit_common.so")' \
      'set(vikit_common_LIBRARY_DIRS "/usr/local/lib")' \
      > /usr/local/lib/cmake/vikit_common/vikit_commonConfig.cmake && \
    ldconfig && \
    touch vikit_common/COLCON_IGNORE vikit_py/COLCON_IGNORE && \
    rm -rf /tmp/vikit_common_build
# ============================================================
# ROS 2 workspace
# ============================================================
COPY ros2_ws /workspace/ros2_ws
RUN source /opt/ros/humble/setup.bash && \
    cd /workspace/ros2_ws && \
    MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --executor sequential \
        --symlink-install \
        --packages-select \
            limo_msgs \
            limo_base \
            livox_ros_driver2 \
            fast_lio \
            vikit_ros \
            fast_livo \
        --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DROS_EDITION=ROS2 \
            -DDISTRO_ROS=humble
# Ambiente automático ao abrir um terminal
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/ros2_ws/install/setup.bash" >> /root/.bashrc
WORKDIR /workspace/ros2_ws
CMD ["bash"]

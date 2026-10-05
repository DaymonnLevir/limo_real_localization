FROM ros:noetic-ros-base-focal
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ARG FAST_CALIB_SHA=1018ecfdf9deda51b91a8a11bd11972a0b159008
ARG LIVOX_DRIVER_SHA=21445540f0d100dc86a7e6df312dd70bbdb4afdf
ARG LIVOX_SDK_SHA=c0796f04c143143899c87a773d9f6b7136453c0b
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates libapr1-dev libopencv-dev libopencv-contrib-dev \
    libeigen3-dev python3-opencv python3-yaml v4l-utils iproute2 \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-cv-bridge ros-noetic-image-geometry \
    && rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/Livox-SDK/Livox-SDK2 /opt/Livox-SDK2 \
    && cd /opt/Livox-SDK2 && git checkout "$LIVOX_SDK_SHA" \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build -j2 && cmake --install build && ldconfig
WORKDIR /opt/fast_calib_ws
RUN mkdir src && git clone https://github.com/hku-mars/FAST-Calib src/fast_calib \
    && cd src/fast_calib && git checkout "$FAST_CALIB_SHA"
RUN git clone https://github.com/Livox-SDK/livox_ros_driver2 src/livox_ros_driver2 \
    && cd src/livox_ros_driver2 && git checkout "$LIVOX_DRIVER_SHA" && cp package_ROS1.xml package.xml
RUN source /opt/ros/noetic/setup.bash && catkin_make -j2 -l2 -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release
COPY entrypoint.sh /stage3_entrypoint.sh
ENV ROS_MASTER_URI=http://127.0.0.1:11321 ROS_IP=127.0.0.1 ROS_HOME=/tmp/ros HOME=/tmp
ENTRYPOINT ["/stage3_entrypoint.sh"]
CMD ["roscore", "-p", "11321"]

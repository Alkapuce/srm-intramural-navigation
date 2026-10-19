FROM osrf/ros@sha256:b0302b39a2321079950b6a91a13eafe62e60bd42ffe61c3cea3d5e96886d0a87
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip libomp-dev libeigen3-dev libapr1-dev liboctomap-dev nlohmann-json3-dev mesa-utils x11-utils x11-apps \
    imagemagick ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
    ros-humble-ros-gz ros-humble-rclcpp ros-humble-rclcpp-action \
    ros-humble-rclcpp-components ros-humble-rclcpp-lifecycle \
    && test "$(dpkg-query -W -f='${Version}' ros-humble-rclcpp | cut -d- -f1)" = 16.0.21 \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir xmacro==1.2.1
COPY cache/small_gicp /tmp/small_gicp
RUN cmake -S /tmp/small_gicp -B /tmp/small_gicp/build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/small_gicp/build -j4 \
    && cmake --install /tmp/small_gicp/build && ldconfig
COPY nav_ws/src /tmp/dependency_ws/src
RUN source /opt/ros/humble/setup.bash && rosdep update --rosdistro humble \
    && apt-get update \
    && rosdep install --from-paths /tmp/dependency_ws/src --ignore-src --rosdistro humble \
       -t build -t buildtool -t build_export -t buildtool_export -t exec -y \
    && rm -rf /var/lib/apt/lists/* /tmp/dependency_ws /tmp/small_gicp
ENV ROS_DOMAIN_ID=71 ROS_LOCALHOST_ONLY=1 QT_X11_NO_MITSHM=1
WORKDIR /work/nav_ws
ENTRYPOINT ["/bin/bash", "/work/scripts/internal/container-entrypoint.sh"]
CMD ["bash"]

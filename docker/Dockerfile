FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    libegl1-mesa-dev \
    libglfw3-dev \
    python3-rosdep \
    ros-jazzy-mujoco-vendor \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/mujoco_ws/src
ARG MUJOCO_ROS2_CONTROL_REV=58f465b1b4634a6f366e71919189574f5d95c1e8
RUN git clone https://github.com/ros-controls/mujoco_ros2_control.git && \
    cd mujoco_ros2_control && git checkout "${MUJOCO_ROS2_CONTROL_REV}"

# Build with the repository root as the Docker context.
COPY . /workspace/mujoco_ws/src/robotnik_mujoco

WORKDIR /workspace/mujoco_ws
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro jazzy && \
    rm -rf /var/lib/apt/lists/* && \
    /bin/bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo 'source /workspace/mujoco_ws/install/setup.bash' >> /root/.bashrc

CMD ["/bin/bash"]

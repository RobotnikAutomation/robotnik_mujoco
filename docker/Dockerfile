FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# 1. Dependencias básicas y gráficas para MuJoCo
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-dev \
    libglew-dev \
    libosmesa6-dev \
    libglfw3-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# 2. Instalar MuJoCo vía PIP
RUN pip3 install mujoco mujoco-python-viewer --break-system-packages

# 3. Exportar las rutas de la librería de PIP para CMake / C++
RUN PYTHON_SITE=$(python3 -c "import site; print(site.getsitepackages()[0])") && \
    echo "export MuJoCo_DIR=${PYTHON_SITE}/mujoco" >> /root/.bashrc && \
    echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:${PYTHON_SITE}/mujoco" >> /root/.bashrc

# 4. Clonar repositorios dentro del workspace
WORKDIR /workspace/ros2_ws/src
RUN git clone https://github.com/ros-controls/mujoco_ros2_control.git -b main
RUN git clone -b RbWatcher_withsensors https://github.com/RobotnikAutomation/robotnik_mujoco.git
RUN git clone -b RbWatcher_withsensors https://github.com/RobotnikAutomation/robotnik_simulation.git

# 5. Instalar los paquetes .deb locales de robotnik_simulation
RUN apt-get update && \
    (dpkg -i /workspace/ros2_ws/src/robotnik_simulation/debs/*.deb || apt-get install -f -y --no-install-recommends) && \
    rm -rf /var/lib/apt/lists/*

# 6. Instalar dependencias restantes de ROS 2 e instalar el workspace
WORKDIR /workspace/ros2_ws
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro jazzy && \
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# 7. Configurar variables de entorno automáticas
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /workspace/ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
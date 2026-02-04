FROM osrf/ros:humble-desktop-full

# Evitar prompts interactivos
ENV DEBIAN_FRONTEND=noninteractive

# 1. Instalar dependencias básicas y gráficas
RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-dev \
    libglew-dev \
    libosmesa6-dev \
    libglfw3-dev \
    wget \
    unzip \
    git \
    && rm -rf /var/lib/apt/lists/*

# 2. Instalar MuJoCo 3.x (Binarios)
RUN mkdir -p /root/.mujoco && \
    wget https://github.com/google-deepmind/mujoco/releases/download/3.4.0/mujoco-3.4.0-linux-x86_64.tar.gz && \
    tar -xf mujoco-3.4.0-linux-x86_64.tar.gz -C /root/.mujoco && \
    rm mujoco-3.4.0-linux-x86_64.tar.gz

# 3. Variables de entorno
ENV MUJOCO_DIR=/root/.mujoco/mujoco-3.4.0
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/root/.mujoco/mujoco-3.4.0/bin:/root/.mujoco/mujoco-3.4.0/lib
ENV PATH=$PATH:/root/.mujoco/mujoco-3.4.0/bin

# 4. Instalar MuJoCo para Python
RUN pip3 install mujoco mujoco-python-viewer

# 5. Crear Workspace y clonar Robotnik
WORKDIR /ros2_ws/src
RUN git clone https://github.com/moveit/mujoco_ros2_control.git
# Aquí puedes añadir los clones de Robotnik
RUN git clone -b RbWatcher_withsensors https://github.com/RobotnikAutomation/robotnik_mujoco.git

WORKDIR /ros2_ws
# 1. Actualizar rosdep e instalar dependencias automáticamente
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro humble

# 2. Ahora sí, compilar
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

CMD ["/bin/bash"]

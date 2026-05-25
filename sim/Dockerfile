# === Base: ROS 2 Jazzy com Desktop (RViz2 incluso) ============================
FROM osrf/ros:jazzy-desktop-full

LABEL maintainer="Matheus Hipólito Carvalho"
LABEL description="TurtleBot4 Simulator (Gazebo Harmonic) on ROS 2 Jazzy"

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# === Utilitários e dev tools ==================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget curl vim net-tools iputils-ping sudo \
    ros-dev-tools python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# === Gazebo Harmonic (conforme documentação Jazzy) ============================
RUN apt-get update && apt-get install -y curl gnupg lsb-release \
 && curl https://packages.osrfoundation.org/gazebo.gpg \
      --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y gz-harmonic \
 && rm -rf /var/lib/apt/lists/*

# === Qt/QML necessários para a GUI do Gazebo ==================================
RUN apt-get update && apt-get install -y \
    qml-module-qtquick-controls qml-module-qtquick-controls2 \
    qml-module-qtgraphicaleffects qml-module-qtquick-layouts \
    qml-module-qtquick-dialogs qml-module-qtquick-shapes \
    qml-module-qt-labs-platform \
 && rm -rf /var/lib/apt/lists/*

# === TurtleBot4 Simulator + Create3 + ROS-Gazebo Bridge =======================
# (já puxa as dependências certas no Jazzy — sem rosdep)
RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlebot4-simulator \
    ros-jazzy-irobot-create-nodes \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup \
    ros-jazzy-teleop-twist-keyboard \
 && rm -rf /var/lib/apt/lists/*

# (Opcional) RViz configs do TB4 para visualizar modelo/robô
# RUN apt-get update && apt-get install -y ros-jazzy-turtlebot4-viz && rm -rf /var/lib/apt/lists/*

# === Ambiente ROS pronto ao abrir o shell =====================================
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# === Entrypoint padrão ========================================================
CMD [ "bash" ]

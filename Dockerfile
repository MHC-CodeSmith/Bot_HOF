# === Base: ROS 2 Humble com Desktop (RViz2 incluso) ===========================
FROM osrf/ros:humble-desktop-full

LABEL maintainer="Matheus Hipólito Carvalho"
LABEL description="TurtleBot4 Simulator (Ignition Fortress) on ROS 2 Humble"

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# === Utilitários e dev tools ==================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    git wget curl vim net-tools iputils-ping sudo \
    ros-dev-tools python3-colcon-common-extensions \
 && rm -rf /var/lib/apt/lists/*

# === Ignition Gazebo (Fortress), conforme documentação =======================
RUN apt-get update && apt-get install -y wget gnupg lsb-release \
 && echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && apt-get update && apt-get install -y ignition-fortress \
 && rm -rf /var/lib/apt/lists/*

# === Qt/QML necessários para a GUI do Ignition (Jammy) =======================
RUN apt-get update && apt-get install -y \
    qml-module-qtquick-controls qml-module-qtquick-controls2 \
    qml-module-qtgraphicaleffects qml-module-qtquick-layouts \
    qml-module-qtquick-dialogs qml-module-qtquick-shapes \
    qml-module-qt-labs-platform \
 && rm -rf /var/lib/apt/lists/*

# === TurtleBot4 Simulator + Create3 + ROS-Ignition Bridge =====================
# (isto já puxa as dependências certas no Humble — sem rosdep)
RUN apt-get update && apt-get install -y \
    ros-humble-turtlebot4-simulator \
    ros-humble-irobot-create-nodes \
    ros-humble-ros-ign-bridge \
 && rm -rf /var/lib/apt/lists/*


 RUN apt-get update && apt-get install -y \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-teleop-twist-keyboard \
  && rm -rf /var/lib/apt/lists/*


  
# (Opcional) RViz configs do TB4 para visualizar modelo/robô
# RUN apt-get update && apt-get install -y ros-humble-turtlebot4-viz && rm -rf /var/lib/apt/lists/*

# === Ambiente ROS pronto ao abrir o shell =====================================
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# === Entrypoint padrão ========================================================
CMD [ "bash" ]

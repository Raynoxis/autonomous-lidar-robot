# MakersPet Mini - Web Navigation Container
# Base: ROS2 Iron officielle
FROM docker.io/osrf/ros:iron-desktop-full

# Métadonnées
LABEL maintainer="MakersPet Web Navigation"
LABEL description="ROS2 Iron container for MakersPet Mini (120mm) with web interface and autonomous exploration"

# Variables d'environnement
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=iron
ENV WORKSPACE=/app/ros_ws
ENV UROS_WS=/app/uros_ws

# Installation des dépendances système
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    wget \
    curl \
    vim \
    nano \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Installation des dépendances Python pour RosBridge
RUN pip3 install --no-cache-dir \
    tornado \
    autobahn \
    pymongo \
    cbor2

# Création des workspaces ROS2
RUN mkdir -p ${WORKSPACE}/src ${UROS_WS}/src
WORKDIR ${WORKSPACE}

# Installation des packages ROS2 nécessaires via apt
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-ament-cmake-mypy \
    && rm -rf /var/lib/apt/lists/*

# Clonage des packages sources nécessaires
WORKDIR ${WORKSPACE}/src

# Explore Lite (exploration autonome) - Fork KaiAI optimisé pour MakersPet
RUN git clone -b main https://github.com/kaiaai/m-explore-ros2.git

# Auto Mapper (cartographie automatique avec exploration)
RUN git clone https://github.com/Omar-Salem/auto_mapper.git

# Packages KaiAI pour télémétrie et messages
RUN git clone -b iron https://github.com/kaiaai/kaiaai_telemetry.git && \
    cd kaiaai_telemetry && \
    git checkout c7e1c25 && \
    cd .. && \
    git clone -b iron https://github.com/kaiaai/kaiaai_msgs.git

# Package MakersPet Mini (robot specifique - 120mm base)
RUN git clone -b iron https://github.com/makerspet/makerspet_mini.git

# Rosbridge Suite pour interface web
RUN git clone -b ros2 https://github.com/RobotWebTools/rosbridge_suite.git

# Installation des dépendances rosdep
WORKDIR ${WORKSPACE}
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build du workspace principal (TOUT en une fois)
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Configuration micro-ROS via micro_ros_setup
WORKDIR ${UROS_WS}/src
RUN git clone -b iron https://github.com/micro-ROS/micro_ros_setup.git

WORKDIR ${UROS_WS}
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build micro_ros_setup
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install"

# Create and build micro-ROS agent
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${UROS_WS}/install/setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh"

# Retour au workspace principal
WORKDIR /app

# Copie des fichiers de configuration
COPY config/ /app/config/
COPY launch/ /app/launch/
COPY web/ /app/web/

# Création des répertoires de données persistantes
RUN mkdir -p /app/maps /app/logs

# Port pour RosBridge WebSocket
EXPOSE 9092

# Port pour serveur web
EXPOSE 8080

# Script d'entrée
COPY scripts/entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

# Point d'entrée
ENTRYPOINT ["/app/entrypoint.sh"]
CMD ["bash"]

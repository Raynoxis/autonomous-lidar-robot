# MakersPet Mini - Web Navigation Container (self-contained)
# Stage 1: build frontend
FROM node:20 AS webbuilder
WORKDIR /webapp
COPY web/package*.json ./
RUN npm ci
COPY web/ ./
RUN npm run build

# Stage 2: runtime ROS2 + frontend assets
FROM docker.io/osrf/ros:iron-desktop-full

LABEL maintainer="MakersPet Web Navigation"
LABEL description="ROS2 Iron container for MakersPet Mini (120mm) with modern React web interface and autonomous exploration"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=iron
ENV WORKSPACE=/app/ros_ws
ENV UROS_WS=/app/uros_ws
ENV NODE_VERSION=20

# Système + Node.js (pour scripts éventuels) + pip deps rosbridge
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
    ca-certificates \
    gnupg \
    && rm -rf /var/lib/apt/lists/* && \
    curl -fsSL https://deb.nodesource.com/setup_${NODE_VERSION}.x | bash - && \
    apt-get update && apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/* && \
    node --version && npm --version

RUN pip3 install --no-cache-dir tornado autobahn pymongo cbor2

# Workspaces ROS2
RUN mkdir -p ${WORKSPACE}/src ${UROS_WS}/src
WORKDIR ${WORKSPACE}

# Packages ROS2 via apt
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

# Clonage des sources
WORKDIR ${WORKSPACE}/src
RUN git clone -b main https://github.com/kaiaai/m-explore-ros2.git
RUN git clone https://github.com/Omar-Salem/auto_mapper.git
RUN git clone -b iron https://github.com/kaiaai/kaiaai_telemetry.git && \
    cd kaiaai_telemetry && git checkout c7e1c25 && cd .. && \
    git clone -b iron https://github.com/kaiaai/kaiaai_msgs.git
RUN git clone -b iron https://github.com/makerspet/makerspet_mini.git
RUN git clone -b ros2 https://github.com/RobotWebTools/rosbridge_suite.git

# rosdep + build workspace
WORKDIR ${WORKSPACE}
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# micro-ROS setup
WORKDIR ${UROS_WS}/src
RUN git clone -b iron https://github.com/micro-ROS/micro_ros_setup.git
WORKDIR ${UROS_WS}
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install" && \
    /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${UROS_WS}/install/setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh"

# Retour /app et configuration
WORKDIR /app
COPY config/ /app/config/
COPY launch/ /app/launch/

# Frontend buildé (dist + scripts)
RUN mkdir -p /app/web
COPY --from=webbuilder /webapp/dist /app/web/dist
COPY --from=webbuilder /webapp/serve.py /app/web/serve.py
COPY --from=webbuilder /webapp/ros_api.py /app/web/ros_api.py
RUN ls -la /app/web && ls -la /app/web/dist

# Répertoires persistants
RUN mkdir -p /app/maps /app/logs

EXPOSE 9092
EXPOSE 8082
EXPOSE 8083

COPY scripts/entrypoint.sh /app/entrypoint.sh
RUN chmod +x /app/entrypoint.sh

ENTRYPOINT ["/app/entrypoint.sh"]
CMD ["bash"]

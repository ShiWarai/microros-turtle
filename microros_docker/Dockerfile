FROM osrf/ros:humble-desktop-full

# Установите зависимости для micro-ROS
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    python3-rosdep \
    python3-colcon-common-extensions \
    git \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-imu-tools \
    && rm -rf /var/lib/apt/lists/*

# Создайте рабочее пространство для micro-ROS Agent в /uros2_ws
WORKDIR /uros2_ws

# Клонируйте micro-ROS в src
RUN git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Копируйте дополнительные пакеты с хоста в /uros2_ws/src
COPY extra_packages/ src/

# Обновите зависимости и соберите micro-ROS вместе с дополнительными пакетами
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    rm -rf /var/lib/apt/lists/*"

# Установите micro-ROS Agent в /uros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /uros2_ws/install/setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh"

# Настройте автоматический source при запуске
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /uros2_ws/install/setup.bash" >> /root/.bashrc

# Создание папки для работы с прошивкой
WORKDIR /root/firmwares
RUN git clone https://github.com/ShiWarai/microros-turtle.git /tmp/microros-turtle && \
    mv /tmp/microros-turtle/firmware_body . && \
    mv /tmp/microros-turtle/firmware_camera . && \
    rm -rf /tmp/microros-turtle

# Переключите рабочую директорию на /ros2_ws
WORKDIR /ros2_ws
FROM osrf/ros:humble-desktop-full

# Установите дополнительные зависимости, если нужно
RUN apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Настройка рабочей директории
WORKDIR /ros2_ws
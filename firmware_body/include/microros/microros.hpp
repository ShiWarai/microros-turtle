#pragma once

#include <Arduino.h>
#include <stdio.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>
#include <std_msgs/msg/string.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <rcl_interfaces/msg/log.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SparkFunMPU9250-DMP.h>

#include "motor_controller/motor_controller.hpp"
#include "settings/settings.hpp"
#include "lidar_controller/dreame_lidar.hpp"
#include "microros_logger.hpp"

#define RANGES_SIZE 360

class MicroRosController
{
public:
    static void microrosTask(void *pvParameters);
    static void onOTAStart();
    static void onOTAProgress(size_t current, size_t final);
    static void onOTAEnd(bool success);
private:
    static unsigned long ota_progress_millis;
};
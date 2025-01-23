#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/quaternion.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SparkFunMPU9250-DMP.h>

#include "motor_controller/motor_controller.hpp"
#include "settings/settings.hpp"
#include "dreame_lidar.hpp"


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ Serial.printf("Error: %d\r\n", temp_rc); error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {}}

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
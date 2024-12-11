#pragma once

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <WiFi.h>
#include <ESPmDNS.h>

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
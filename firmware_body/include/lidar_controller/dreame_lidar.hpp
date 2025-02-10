#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define LIDAR_DATA_SIZE 360
#define LIDAR_BAUDRATE 115200

class DreameLidar {
public:
    DreameLidar(HardwareSerial *serial, int dataSize = LIDAR_DATA_SIZE, bool isInvert = true);
    static void lidarTask(void* param);
    
    int dataSize;
    float* theta;
    uint16_t* distance;
    uint8_t* intensity;
    float rpm;

    bool dataObtained;
    SemaphoreHandle_t dataLock;
private:
    void getDataUnit();

    HardwareSerial *serial;
    int writePos;
    bool isInvert;
};
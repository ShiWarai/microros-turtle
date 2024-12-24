#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define LIDAR_DATA_SIZE 360

class DreameLidar {
public:
    DreameLidar(HardwareSerial *serial, int dataSize = LIDAR_DATA_SIZE, bool isInvert = true);
    void startTask();
    
    int dataSize;
    float* theta;
    uint16_t* distance;
    uint8_t* intensity;
    float rpm;

    bool dataObtained;
    SemaphoreHandle_t lock;
private:
    static void getData(void* param);
    void getDataUnit();

    HardwareSerial *serial;
    int writePos;
    bool isInvert;
};
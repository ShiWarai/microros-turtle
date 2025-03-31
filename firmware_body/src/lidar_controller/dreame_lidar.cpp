#include "lidar_controller/dreame_lidar.hpp"

DreameLidar::DreameLidar(HardwareSerial *serial, int dataSize, bool isInvert)
    : serial(serial), dataSize(dataSize), isInvert(isInvert), writePos(0), dataObtained(false), rpm(0) {
    theta = new float[dataSize];
    distance = new uint16_t[dataSize];
    intensity = new uint8_t[dataSize];

    this->serial->begin(LIDAR_BAUDRATE);
    dataLock = xSemaphoreCreateMutex();
}

void DreameLidar::getData() {
    // Чтение заголовка
    uint8_t header[] = {0x55, 0xAA, 0x03, 0x08};
    int headerPos = 0;

    while (true) {
        uint8_t tmp;
        this->serial->readBytes(&tmp, 1);

        if (tmp == header[headerPos]) {
            headerPos++;
            if (headerPos == sizeof(header))
                break;
        } else {
            headerPos = 0;
        }
    }

    // Чтение скорости, углов
    uint16_t rotationSpeedTmp, startAngleTmp;
    this->serial->readBytes((uint8_t*)&rotationSpeedTmp, 2);
    this->serial->readBytes((uint8_t*)&startAngleTmp, 2);
    rpm = rotationSpeedTmp / 64.0;
    float startAngle = (startAngleTmp - 0xA000) / 64.0;

    // Чтение дистанции
    uint16_t distanceTmp[8];
    uint8_t intensityTmp[8];
    for (int i = 0; i < 8; i++) {
        this->serial->readBytes((uint8_t*)&distanceTmp[i], 2);
        this->serial->readBytes((uint8_t*)&intensityTmp[i], 1);
    }

    // Чтения окончания пакета
    uint16_t endAngleTmp, crc;
    this->serial->readBytes((uint8_t*)&endAngleTmp, 2);
    this->serial->readBytes((uint8_t*)&crc, 2);
    float endAngle = (endAngleTmp - 0xA000) / 64.0;

    // char str[128];
    // sprintf(str, "start_angle: %.3f, end_angle: %.3f", startAngle, endAngle);
    // MicroROSLogger::log(str, "getData()", "dreame_lidar.cpp", LogLevel::INFO, false);

    // Обновление данных
    if (xSemaphoreTake(dataLock, portMAX_DELAY) == pdTRUE) {
        if(endAngle < startAngle)
            endAngle += 360;

        if(startAngle - preStartAngle < 0)
            this->dataObtained = true;
        preStartAngle = startAngle;

        float startAngleRad = (startAngle + angle_offset) * M_PI / 180 * (isInvert ? -1 : 1);
        float endAngleRad = (endAngle + angle_offset) * M_PI / 180 * (isInvert ? -1 : 1);
        float angleIncrement = (endAngleRad - startAngleRad) / 8.0;

        for (int i = 0; i < 8; i++) {
            theta[writePos] = startAngleRad + angleIncrement * i;
            distance[writePos] = distanceTmp[i];
            intensity[writePos] = intensityTmp[i];
            
            writePos++;

            if (writePos >= dataSize)
                writePos = 0;
        }
        xSemaphoreGive(dataLock);
    }
}

void DreameLidar::lidarTask(void* param) {
    DreameLidar* self = static_cast<DreameLidar*>(param);
    
    while (true) {
        self->getData();

        vTaskDelay(1);
    }
}
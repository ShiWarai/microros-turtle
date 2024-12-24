#include "microros/dreame_lidar.hpp"

DreameLidar::DreameLidar(HardwareSerial *serial, int dataSize, bool isInvert)
    : serial(serial), dataSize(dataSize), isInvert(isInvert), writePos(0), dataObtained(false), rpm(0) {
    theta = new float[dataSize];
    distance = new uint16_t[dataSize];
    intensity = new uint8_t[dataSize];

    this->serial->begin(115200);
    lock = xSemaphoreCreateMutex();
}

void DreameLidar::getDataUnit() {
    // Reading header
    uint8_t header[] = {0x55, 0xAA, 0x03, 0x08};
    int headerPos = 0;
    while (true) {
        uint8_t tmp;
        this->serial->readBytes(&tmp, 1);
        if (tmp == header[headerPos]) {
            headerPos++;
            if (headerPos == sizeof(header)) {
                break;
            }
        } else {
            headerPos = 0;
        }
    }

    // Read rotation speed and start angle
    uint16_t rotationSpeedTmp, startAngleTmp;
    this->serial->readBytes((uint8_t*)&rotationSpeedTmp, 2);
    this->serial->readBytes((uint8_t*)&startAngleTmp, 2);
    rpm = rotationSpeedTmp / 64.0;
    float startAngle = (startAngleTmp - 0xA000) / 64.0;

    uint16_t distanceTmp[8];
    uint8_t intensityTmp[8];
    for (int i = 0; i < 8; i++) {
        uint8_t data[3];
        this->serial->readBytes(data, 3);
        distanceTmp[i] = data[0] | (data[1] << 8);
        
        intensityTmp[i] = data[2];
    }

    uint16_t endAngleTmp, crc;
    this->serial->readBytes((uint8_t*)&endAngleTmp, 2);
    this->serial->readBytes((uint8_t*)&crc, 2);
    float endAngle = (endAngleTmp - 0xA000) / 64.0;

    // Update data
    if (xSemaphoreTake(lock, portMAX_DELAY) == pdTRUE) {
        float startAngleRad = startAngle * M_PI / 180 * (isInvert ? -1 : 1);
        float endAngleRad = endAngle * M_PI / 180 * (isInvert ? -1 : 1);
        float angleIncrement = (endAngleRad - startAngleRad) / 8.0;
        for (int i = 0; i < 8; i++) {
            theta[writePos] = startAngleRad + angleIncrement * i;
            distance[writePos] = distanceTmp[i];
            #ifdef LIDAR_INTENSITY
            intensity[writePos] = intensityTmp[i];
            #endif
            writePos++;
            if (writePos >= dataSize) {
                writePos = 0;
            }
        }
        xSemaphoreGive(lock);
    }
}

void DreameLidar::getData(void* param) {
    DreameLidar* self = static_cast<DreameLidar*>(param);
    float preStartAngle = 0;
    
    while (true) {
        self->getDataUnit();
        if (self->theta[0] < preStartAngle) {
            self->dataObtained = true;
        }
        preStartAngle = self->theta[0];

        vTaskDelay(1);
    }
}

void DreameLidar::startTask() {
    xTaskCreate(getData, "lidar_data_task", 8196, this, 1, NULL);

    while(!this->dataObtained)
        vTaskDelay(100);
}
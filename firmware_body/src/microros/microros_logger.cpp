#include "microros/microros_logger.hpp"

QueueHandle_t MicroROSLogger::logQueue = nullptr;
SemaphoreHandle_t MicroROSLogger::logMutex = nullptr;

// Инициализация очереди и мьютекса
void MicroROSLogger::init() {
    logQueue = xQueueCreate(QUEUE_LENGTH, sizeof(LogMessage));
    logMutex = xSemaphoreCreateMutex();
}

// Добавление сообщения в очередь
void MicroROSLogger::log(String message, String func, String file, LogLevel level, bool to_serial_too) {
    LogMessage logMessage;

    logMessage.level = level;
    strcpy(logMessage.message, message.c_str());
    strcpy(logMessage.func, func.c_str());
    strcpy(logMessage.file, file.c_str());

    if(to_serial_too)
        Serial.printf("[%s] from %s: %s\r\n", logMessage.file, logMessage.func, logMessage.message);

    if (logMutex != nullptr && xSemaphoreTake(logMutex, portMAX_DELAY)) {
        xQueueSend(logQueue, &logMessage, 0);
        xSemaphoreGive(logMutex);
    }
}

// Получение следующего сообщения из очереди
LogMessage MicroROSLogger::getNextLogMessage() {
    LogMessage logMessage;

    if (logMutex != nullptr && xSemaphoreTake(logMutex, portMAX_DELAY)) {
        xQueueReceive(logQueue, &logMessage, 0);
        xSemaphoreGive(logMutex);
    }

    return logMessage;
}

// Проверка наличия сообщений в очереди
bool MicroROSLogger::hasLogMessages() {
    bool hasMessages = false;

    if (logMutex != nullptr && xSemaphoreTake(logMutex, portMAX_DELAY)) {
        hasMessages = (uxQueueMessagesWaiting(logQueue) > 0);
        xSemaphoreGive(logMutex);
    }

    return hasMessages;
}

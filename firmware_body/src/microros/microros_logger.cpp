#include "microros/microros_logger.hpp"

QueueHandle_t MicroROSLogger::logQueue = nullptr;
SemaphoreHandle_t MicroROSLogger::logMutex = nullptr;

// Инициализация очереди и мьютекса
void MicroROSLogger::init() {
    logQueue = xQueueCreate(QUEUE_LENGTH, sizeof(LogMessage));
    logMutex = xSemaphoreCreateMutex();
}

// Добавление сообщения в очередь
void MicroROSLogger::log(const String &message, LogLevel level) {
    LogMessage logMessage = {level, message};

    if (logMutex != nullptr && xSemaphoreTake(logMutex, portMAX_DELAY)) {
        if (xQueueSend(logQueue, &logMessage, 0) != pdPASS) {
            Serial.println("Failed to send message to queue!");
        }
        xSemaphoreGive(logMutex);
    }
}

// Получение следующего сообщения из очереди
LogMessage MicroROSLogger::getNextLogMessage() {
    LogMessage logMessage = {LogLevel::DEBUG, ""};

    if (logMutex != nullptr && xSemaphoreTake(logMutex, portMAX_DELAY)) {
        if (xQueueReceive(logQueue, &logMessage, 0) == pdPASS) {
            // Сообщение успешно получено
        } else {
            Serial.println("No messages in queue!");
        }
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

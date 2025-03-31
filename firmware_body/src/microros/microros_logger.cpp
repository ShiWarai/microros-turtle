#include "microros/microros_logger.hpp"

QueueHandle_t MicroROSLogger::log_queue = nullptr;
SemaphoreHandle_t MicroROSLogger::log_mutex = nullptr;

void MicroROSLogger::init() {
    log_queue = xQueueCreate(QUEUE_LENGTH, sizeof(LogMessage));
    log_mutex = xSemaphoreCreateMutex();
}

bool MicroROSLogger::log(String message_text, String func, String file, LogLevel level, bool to_serial_too) {
    if(settings.logs_enabled) {
        LogMessage logMessage;

        logMessage.level = level;
        strcpy(logMessage.message, message_text.c_str());
        strcpy(logMessage.func, func.c_str());
        strcpy(logMessage.file, file.c_str());

        return MicroROSLogger::log(logMessage, to_serial_too);
    } else {
        return false;
    }
}

bool MicroROSLogger::log(LogMessage msg, bool to_serial_too) {
    if(settings.logs_enabled)
    {
        if(to_serial_too)
            Serial.printf("[%s] from %s: %s\r\n", msg.file, msg.func, msg.message);

        if (log_mutex != nullptr && xSemaphoreTake(log_mutex, portMAX_DELAY)) {
            xQueueSend(log_queue, &msg, 0);
            xSemaphoreGive(log_mutex);
            return true;
        }
    }

    return false;
}

LogMessage MicroROSLogger::get_next_log_message() {
    LogMessage log_message;

    if (log_mutex != nullptr && xSemaphoreTake(log_mutex, portMAX_DELAY)) {
        xQueueReceive(log_queue, &log_message, 0);
        xSemaphoreGive(log_mutex);
    }

    return log_message;
}

bool MicroROSLogger::has_log_messages() {
    bool hasMessages = false;

    if (log_mutex != nullptr && xSemaphoreTake(log_mutex, portMAX_DELAY)) {
        hasMessages = (uxQueueMessagesWaiting(log_queue) > 0);
        xSemaphoreGive(log_mutex);
    }

    return hasMessages;
}

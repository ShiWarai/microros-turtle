#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "settings/settings.hpp"

#define QUEUE_LENGTH 10
#define MESSAGE_LENGTH 256
#define FUNC_LENGTH 32
#define FILE_LENGTH 32

enum class LogLevel {
    UNSET = 0,
    DEBUG = 10,
    ERROR = 40,
    FATAL = 50,
    INFO = 20,
    WARN = 30
};

struct LogMessage {
    LogLevel level;
    char file[FILE_LENGTH];
    char func[FUNC_LENGTH];
    char message[MESSAGE_LENGTH];
};

class MicroROSLogger {
public:
    static void init(); // Инициализация очереди и мьютекса
    static void log(String message, String func = "", String file = "", LogLevel level = LogLevel::INFO, bool to_serial_too = true);
    static LogMessage getNextLogMessage();
    static bool hasLogMessages();

private:
    static QueueHandle_t logQueue; // Очередь FreeRTOS
    static SemaphoreHandle_t logMutex; // Мьютекс для синхронизации
};
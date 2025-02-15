#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#define QUEUE_LENGTH 10

enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

struct LogMessage {
    LogLevel level;
    String message;
};

class MicroROSLogger {
public:
    static void init(); // Инициализация очереди и мьютекса
    static void log(const String &message, LogLevel level = LogLevel::INFO);
    static LogMessage getNextLogMessage();
    static bool hasLogMessages();

private:
    static QueueHandle_t logQueue; // Очередь FreeRTOS
    static SemaphoreHandle_t logMutex; // Мьютекс для синхронизации
};
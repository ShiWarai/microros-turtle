#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "settings/settings.hpp"

#define QUEUE_LENGTH 20
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
    static void init();
    static bool log(String message_text, String func = "", String file = "", LogLevel level = LogLevel::INFO, bool to_serial_too = true);
    static bool log(LogMessage msg, bool to_serial_too = true);
    static LogMessage get_next_log_message();
    static bool has_log_messages();

private:
    static QueueHandle_t log_queue;
    static SemaphoreHandle_t log_mutex;
};
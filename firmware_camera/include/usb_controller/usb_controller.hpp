#pragma once

#include <Arduino.h>
#include <nvs_flash.h>
#include <Preferences.h>
#include <WiFi.h>
#include "settings/settings.hpp"

#define DECLARE_SERIAL_PRINT_ITER(TYPE, F1, F2, SETTING_POINTER) \
if(SETTINGS_INFO[i].type == #TYPE) \
{ \
    Serial.print(SETTINGS_INFO[i].name); \
    Serial.print("\r\t\t\t="); \
    Serial.print(*(std::get<TYPE*>(SETTING_POINTER))); \
}

#define GEN_SETTINGS_OUTPUT_DEFAULT(SETTING_POINTER, TYPES) \
for (unsigned short i = 0; i < SETTING_TYPE::SETTINGS_COUNT; i++) { \
    SETTING_POINTER = getSettingFieldPointer(i); \
    Serial.print(String(i+1)+") "); \
    TYPES(DECLARE_SERIAL_PRINT_ITER, SETTING_POINTER) \
    Serial.println(); \
}

#define GENERATE_UNIQUE_BUFFERS(TYPE, F1, F2, ...) \
TYPE buffer_##TYPE;

#define GENERATE_SERIAL_INPUT_CASE(TYPE, NAME, REBOOT_IS_REQUIRED, INPUT_VALIDATOR, UPDATE_QUEUE, UPDATE) \
case NAME: \
    Serial.printf("\r\nВведите новый %s: ", SETTINGS_INFO[NAME].name);\
    if(read_##TYPE(&buffer_##TYPE, SETTINGS_INFO[NAME].input_validator) != 0) { \
        Serial.println("\r\nОшибка ввода"); \
        break; \
    } \
    update.value = buffer_##TYPE; \
    update.key = NAME; \
    xQueueSend(UPDATE_QUEUE, &UPDATE, portMAX_DELAY); \
    if(SETTINGS_INFO[NAME].reboot_is_required) { \
        vTaskDelay(1000); \
        ESP.restart(); \
    } else \
        vTaskDelay(100); \
    break;

#define GENERATE_SERIAL_INPUTS(UPDATE_QUEUE, UPDATE, FILEDS) \
UNIQUE_SETTINGS_TYPES(GENERATE_UNIQUE_BUFFERS) \
read_uint32_t(&buffer_uint32_t); \
switch (buffer_uint32_t-1) { \
    FILEDS(GENERATE_SERIAL_INPUT_CASE, UPDATE_QUEUE, UPDATE) \
    default: \
        clearInputBuffer(); \
        return; \
}

class UsbController
{
public:
    static void usbTask(void *pvParameters);
private:
    static void clearInputBuffer();
    static void comMenu();
    static void settingsMenu();
    static void outputInfo();

    static String readInput();
    static error_t read_String(String *, error_t validator(String));
    static error_t read_uint32_t(uint32_t *, error_t validator(String));
    static error_t read_float(float *, error_t validator(String));
};


#pragma once

#include <Arduino.h>
#include <nvs_flash.h>
#include <Preferences.h>
#include <WiFi.h>
#include "settings/settings.hpp"

class UsbController
{
public:
    static void usbTask(void *pvParameters);
private:
    static void clearInputBuffer();
    static void comMenu();
    static void settingsMenu();
    static void printSystemInfo();

    static String readInput();
    static error_t read_String(String *, error_t validator(String));
    static error_t read_uint32_t(uint32_t *, error_t validator(String));
    static error_t read_float(float *, error_t validator(String));
};


#pragma once

#include <ArduinoJson.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <ESPmDNS.h>
#ifdef CAMERA
    #include "webserver/AsyncWebServer.hpp"
#endif

#include "settings/settings.hpp"

class WirelessController
{
public:
    static void wirelessTask(void *pvParameters);
private:
    static bool deserializeSettings(String json_str, bool &needReboot);
    static String serializeSettings();
    static String serializeTestingResult();
};
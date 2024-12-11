#pragma once

#include <Arduino.h>
#include "microros/microros.hpp"
#include "webserver/webserver.hpp"
#include "settings/preferences_controller.hpp"

// Functions
void setup();
void loop() {vTaskDelete(NULL);}
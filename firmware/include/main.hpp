#pragma once

#include <Arduino.h>
#include <nvs_flash.h>
#include "microros/microros.hpp"
#include "webserver/webserver.hpp"
#include "settings/preferences_controller.hpp"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

void setup();
void loop() {vTaskDelete(NULL);}
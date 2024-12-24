#pragma once

#include <Arduino.h>
#include <nvs_flash.h>
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include "microros/microros.hpp"
#include "webserver/webserver.hpp"
#include "settings/preferences_controller.hpp"
#include "usb_controller/usb_controller.hpp"


void setup();
void loop() {vTaskDelete(NULL);}
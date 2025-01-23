#include "main.hpp"
#include "math.h"
#include <SparkFunMPU9250-DMP.h>

void setup()
{
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // Отключение детекции падения тока
  Serial.begin(CONFIG_CONSOLE_UART_BAUDRATE); // Временно до появление логов

  xTaskCreate(PreferencesController::preferencesTask, "Preferences task", 4096, NULL, 1, NULL);

  while (settings.turtle_id == 0) // Ожидаем загрузки настроек в ОЗУ
    vTaskDelay(100);

  xTaskCreate(WirelessController::wirelessTask, "Wireless task", 20000, NULL, 2, NULL);
  xTaskCreate(UsbController::usbTask, "USB task", 4096, NULL, 2, NULL);

  while (WiFi.status() != WL_CONNECTED)
    delay(500);

  if (settings.ros_enabled != 0)
    xTaskCreate(MicroRosController::microrosTask, "microROS task", 8196, NULL, 2, NULL);
}
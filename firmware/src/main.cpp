#include "main.hpp"

void setup()
{
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Отключение детекции падения тока
	Serial.begin(CONFIG_CONSOLE_UART_BAUDRATE); // Временно до появление логов

	// nvs_flash_erase(); 
    // nvs_flash_init(); 

	xTaskCreate(PreferencesController::preferencesTask, "Preferences task", 4096, NULL, 1, NULL);

	while(settings.turtle_id == 0) // Ожидаем загрузки настроек в ОЗУ
		vTaskDelay(100);

	xTaskCreate(WirelessController::wirelessTask, "Wireless task", 20000, NULL, 2, NULL);

	while (WiFi.status() != WL_CONNECTED)
		delay(500);

	xTaskCreate(MicroRosController::microrosTask, "microROS task", 8196, NULL, 2, NULL);
}
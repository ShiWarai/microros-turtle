#include "main.hpp"

void setup()
{
	Serial.begin(115200); // Временно до появление логов

	xTaskCreate(PreferencesController::preferencesTask, "Preferences task", 4096, NULL, 1, NULL);

	while(settings.turtle_id == 0) // Ожидаем загрузки настроек в ОЗУ
		vTaskDelay(100);

	xTaskCreate(WirelessController::wirelessTask, "Wireless task", 16384, NULL, 2, NULL);

    //WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
	while (WiFi.status() != WL_CONNECTED)
		delay(500);

	xTaskCreate(MicroRosController::microrosTask, "microROS task", 4096, NULL, 2, NULL);
}
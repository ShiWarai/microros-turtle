#pragma once

#include <Arduino.h>

inline unsigned long ota_progress_millis = 0;

static void onOTAStart() {
	Serial.println("OTA: Обновление запущено!");
}

static void onOTAProgress(size_t current, size_t final) {
	if (millis() - ota_progress_millis > 1000) {
		ota_progress_millis = millis();
		Serial.printf("OTA: Загружено %u байт из конечных %u байт\r\n", current, final);
	}
}

static void onOTAEnd(bool success) {
	if (success) {
		Serial.println("OTA: Обновление завершено успешно!");

		xTaskCreate([](void *){vTaskDelay(3000); ESP.restart();}, "restartOTATask", 1024, NULL, 0, NULL);
	} else {
		Serial.println("OTA: Произошли ошибки при обновлении!");
	}
}

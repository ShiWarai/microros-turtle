#include "webserver/webserver.hpp"

#define GENERATE_UNIQUE_BUFFERS(TYPE, F1, F2, ...) \
TYPE buffer_##TYPE;

#define DECLARE_DESERIALIZE_ITER(TYPE, NAME, REBOOT_IS_REQUIRED, INPUT_VALIDATOR, SOURCE_STR, JSON_NAME, SETTINGS_NAME, UPDATE_QUEUE, NEED_REBOOT) \
SettingUpdate update_##NAME; \
if(json[#NAME].is<TYPE>()) { \
	buffer_##TYPE = JSON_NAME[#NAME].as<TYPE>(); \
    if(SETTINGS_INFO[NAME].input_validator == nullptr || SETTINGS_INFO[NAME].input_validator(String(buffer_##TYPE)) == 0) { \
		update_##NAME.key = SETTING_TYPE::NAME; \
		update_##NAME.value = buffer_##TYPE; \
		xQueueSend(UPDATE_QUEUE, &update_##NAME, portMAX_DELAY); \
		if(SETTINGS_INFO[NAME].reboot_is_required) \
        	NEED_REBOOT = true; \
	} \
}

#define DECLARE_SERIALIZE_ITER(TYPE, NAME, REBOOT_IS_REQUIRED, INPUT_VALIDATOR, JSON_NAME, SETTINGS_NAME) \
JSON_NAME[#NAME] = SETTINGS_NAME.NAME;

#define GENERATE_DESERIALIZE_CYCLE(SOURCE_STR, JSON_NAME, SETTINGS_NAME, UPDATE_QUEUE, NEED_REBOOT, FIELDS) \
UNIQUE_SETTINGS_TYPES(GENERATE_UNIQUE_BUFFERS) \
FIELDS(DECLARE_DESERIALIZE_ITER, SOURCE_STR, JSON_NAME, SETTINGS_NAME, UPDATE_QUEUE, NEED_REBOOT) \

#define GENERATE_SERIALIZE_CYCLE(JSON_NAME, SETTINGS_NAME, FIELDS) \
FIELDS(DECLARE_SERIALIZE_ITER, JSON_NAME, SETTINGS_NAME)

unsigned long ota_progress_millis = 0;
int without_net_ti = 20000;

void onOTAStart() {
	Serial.println("OTA: Обновление запущено!");
}

void onOTAProgress(size_t current, size_t final) {
	if (millis() - ota_progress_millis > 1000) {
		ota_progress_millis = millis();
		Serial.printf("OTA: Загружено %u байт из конечных %u байт\r\n", current, final);
	}
}

void onOTAEnd(bool success) {
	if (success) {
		Serial.println("OTA: Обновление завершено успешно!");
	} else {
		Serial.println("OTA: Произошли ошибки при обновлении!");
	}
}


bool WirelessController::deserializeSettings(String json_str, bool &needReboot)
{
	JsonDocument json;

	if (deserializeJson(json, json_str))
		return false;

	// Удаление скрытых полей
	json.remove(SETTINGS_INFO[SETTING_TYPE::access_key].name);
	json.remove(SETTINGS_INFO[SETTING_TYPE::influxdb_token].name);

	GENERATE_DESERIALIZE_CYCLE(json_str, json, settings, settingUpdateQueue, needReboot, SETTINGS_FIELDS)
	vTaskDelay(100);

	return true;
}

String WirelessController::serializeSettings()
{
	String json_str;
	JsonDocument json;

	GENERATE_SERIALIZE_CYCLE(json, settings, SETTINGS_FIELDS)

	// Удаление скрытых полей
	json.remove(SETTINGS_INFO[SETTING_TYPE::access_key].name);
	json.remove(SETTINGS_INFO[SETTING_TYPE::influxdb_token].name);
	
	serializeJson(json, json_str);
	return json_str;
}

void WirelessController::wirelessTask(void *pvParameters)
{
	String device_hostname = String("turtle_") + String(settings.turtle_id);

	WiFi.setHostname(device_hostname.c_str()); 
	WiFi.begin(settings.wifi_ssid, settings.wifi_password);

	while (WiFi.status() != WL_CONNECTED) {
		vTaskDelay(500);
        Serial.println("try");
    }

	while (!MDNS.begin(device_hostname.c_str()))
		vTaskDelay(500);
	MDNS.addService("http", "tcp", 80);

	
	AsyncWebServer server(PORT); // Веб-сервер

	DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
 	DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT");
  	DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

	// Получение главной страницы
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->redirect(settings.hostname); });

	// Тестовое получение данных для пинга по HTTP
	server.on("/ping", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send(200, "application/json", "{\"message\":\"OK\"}"); }
	);

	// Получение настроек
	server.on("/settings", HTTP_GET, 
		[](AsyncWebServerRequest *request) {
			if (request->hasHeader("api_key") && request->header("api_key") == settings.access_key) 
			{
				request->send(200, "application/json", WirelessController::serializeSettings()); 
			} 
			else { 
				request->send(403, "application/json", "{\"error\":\"Invalid API key\"}"); 
			}
		}
	);

	// Запись настроек
	server.on("/settings", HTTP_POST, [](AsyncWebServerRequest *request) {}, 
		nullptr,
		[](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
			if (request->hasHeader("api_key") && request->header("api_key") == settings.access_key)
			{
				String body_str(data, len);
				bool needReboot;

				if (WirelessController::deserializeSettings(body_str, needReboot)) {
					if(!needReboot)
						request->send(200, "application/json", "{\"message\":\"Settings updated\"}");
					else {
						request->send(200, "application/json", "{\"message\":\"Restart to update settings...\"}");
						xTaskCreate([](void *){vTaskDelay(1000); ESP.restart();}, "restartTask", 512, NULL, 0, NULL);
					}
				}
				else
					request->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
			}
			else
				request->send(403, "application/json", "{\"error\":\"Invalid access key\"}");
		}
	);

	// Рестарт
	server.on("/restart", HTTP_POST,
		[](AsyncWebServerRequest *request) {
			if (request->hasHeader("api_key") && request->header("api_key") == settings.access_key) {
				request->send(200, "application/json", "{\"message\":\"Restart ESP32...\"}");
				xTaskCreate([](void *){vTaskDelay(1000); ESP.restart();}, "restartTask", 512, NULL, 0, NULL);
			}
			else
				request->send(403, "application/json", "{\"error\":\"Invalid access key\"}");
		}
	);


	// OTA
	ElegantOTA.begin(&server);

	String user = String("turtle_") + 1;
	String password = "1234";
	ElegantOTA.setAuth(user.c_str(), password.c_str());
	ElegantOTA.setAutoReboot(true);
	ElegantOTA.onStart(onOTAStart);
	ElegantOTA.onProgress(onOTAProgress);
	ElegantOTA.onEnd(onOTAEnd);

	server.begin(); // Запускаем сервер

	while(true) {
		ElegantOTA.loop();

		vTaskDelay(1000);
	}
}
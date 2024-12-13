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


#define PART_BOUNDARY "123456789000000000000987654321"
 
// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM
 
// Not tested with this model
//#define CAMERA_MODEL_WROVER_KIT
 
#if defined(CAMERA_MODEL_WROVER_KIT)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    21
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      19
  #define Y4_GPIO_NUM      18
  #define Y3_GPIO_NUM       5
  #define Y2_GPIO_NUM       4
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22
 
#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       32
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21
 
#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
  #define PWDN_GPIO_NUM     -1
  #define RESET_GPIO_NUM    15
  #define XCLK_GPIO_NUM     27
  #define SIOD_GPIO_NUM     25
  #define SIOC_GPIO_NUM     23
  
  #define Y9_GPIO_NUM       19
  #define Y8_GPIO_NUM       36
  #define Y7_GPIO_NUM       18
  #define Y6_GPIO_NUM       39
  #define Y5_GPIO_NUM        5
  #define Y4_GPIO_NUM       34
  #define Y3_GPIO_NUM       35
  #define Y2_GPIO_NUM       17
  #define VSYNC_GPIO_NUM    22
  #define HREF_GPIO_NUM     26
  #define PCLK_GPIO_NUM     21
 
#elif defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif
 
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";


unsigned long ota_progress_millis = 0;

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
	
	serializeJson(json, json_str);
	return json_str;
}

static esp_err_t stream_handler(AsyncWebServerRequest *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];
 
//   res = req;
//   if(res != ESP_OK){
//     return res;
//   }
 
//   while(true){
//     fb = esp_camera_fb_get();
//     if (!fb) {
//       Serial.println("Camera capture failed");
//       res = ESP_FAIL;
//     } else {
//       if(fb->width > 400){
//         if(fb->format != PIXFORMAT_JPEG){
//           bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
//           esp_camera_fb_return(fb);
//           fb = NULL;
//           if(!jpeg_converted){
//             Serial.println("JPEG compression failed");
//             res = ESP_FAIL;
//           }
//         } else {
//           _jpg_buf_len = fb->len;
//           _jpg_buf = fb->buf;
//         }
//       }
//     }
//     if(res == ESP_OK){
//       size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
//       res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
//     }
//     if(res == ESP_OK){
//       res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
//     }
//     if(res == ESP_OK){
//       res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
//     }
//     if(fb){
//       esp_camera_fb_return(fb);
//       fb = NULL;
//       _jpg_buf = NULL;
//     } else if(_jpg_buf){
//       free(_jpg_buf);
//       _jpg_buf = NULL;
//     }
//     if(res != ESP_OK){
//       break;
//     }
//     //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
//   }
  return res;
}

void WirelessController::wirelessTask(void *pvParameters)
{
	String device_hostname = String("turtle_") + String(settings.turtle_id);

	WiFi.setHostname(device_hostname.c_str()); 
	WiFi.begin(settings.wifi_ssid, settings.wifi_password);

	while (WiFi.status() != WL_CONNECTED)
		vTaskDelay(500);

	while (!MDNS.begin(device_hostname.c_str()))
		vTaskDelay(500);
	MDNS.addService("http", "tcp", 80);

	
	AsyncWebServer server(PORT); // Веб-сервер

	DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
 	DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT");
  	DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

	// Получение камеры
	server.on("/webcam", HTTP_GET, stream_handler);

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

	String user = String("turtle_") + settings.turtle_id;
	String password = settings.access_key;
	ElegantOTA.setAuth(user.c_str(), password.c_str());
	ElegantOTA.setAutoReboot(true);
	ElegantOTA.onStart(onOTAStart);
	ElegantOTA.onProgress(onOTAProgress);
	ElegantOTA.onEnd(onOTAEnd);

	server.begin(); // Запускаем сервер

	while(true) {
		ElegantOTA.loop();

		vTaskDelay(settings.wireless_delay);
	}
}
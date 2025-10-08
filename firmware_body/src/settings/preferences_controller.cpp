#include "settings/preferences_controller.hpp"

void PreferencesController::preferencesTask(void *pvParameters) {
    
	Preferences preferences;
	SettingUpdate update;
	
	preferences.begin(SETTINGS_SPACE_NAME, false);

	// Инициализация дефолтных значений
	settings.wifi_password = WIFI_PASSWORD;
	settings.wifi_ssid = WIFI_SSID;
	settings.usb_delay = 1000 / portTICK_PERIOD_MS;
	settings.access_key = ACCESS_KEY;
	settings.ros_enabled = 1;
	settings.logs_enabled = 1;
	#ifdef AGENT_IP
	settings.agent_ip = AGENT_IP;
	#else
	settings.agent_ip = AGENT_HOSTNAME;
	#endif
	settings.agent_port = AGENT_PORT;
	settings.odom_delay = 100;
	settings.lidar_delay = 150;
	settings.imu_delay = 100;
	settings.logger_delay = 50;

	settings.turtle_id = ID;

	DECLARE_SETTING_TYPES_LINKS_VARIANT(UNIQUE_SETTINGS_TYPES) setting;
	void* buffer;

	if(!preferences.isKey(SETTINGS_INFO[SETTING_TYPE::turtle_id].name)) { // Первичная инициализация настроек
		GEN_PUTS_DEFAULT(preferences, buffer, setting, UNIQUE_SETTINGS_TYPES)
	} else { // Загрузка настроек
		GEN_READ_SETTINGS_CYCLE(preferences, buffer, setting, UNIQUE_SETTINGS_TYPES)
	}

	preferences.end();

	while(true) {
		if (xQueueReceive(settingUpdateQueue, &update, portMAX_DELAY)) {
			preferences.begin(SETTINGS_SPACE_NAME, false);
			GEN_UPDATE_ITER(preferences, buffer, setting, update, UNIQUE_SETTINGS_TYPES)
			preferences.end();
		}
		
		vTaskDelay(100);
	}
}
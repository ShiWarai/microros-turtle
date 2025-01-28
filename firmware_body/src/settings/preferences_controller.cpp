#include "settings/preferences_controller.hpp"

void PreferencesController::preferencesTask(void *pvParameters) {
    
	Preferences preferences;
	SettingUpdate update;
	
	preferences.begin(SETTINGS_SPACE_NAME, false);

	// Инициализация дефолтных значений
	settings.turtle_id = ID;
	settings.wifi_password = WIFI_PASSWORD;
	settings.wifi_ssid = WIFI_SSID;
	settings.usb_delay = 1000 / portTICK_PERIOD_MS;
	settings.access_key = ACCESS_KEY;

	settings.ros_enabled = 1;
	settings.agent_ip = AGENT_IP;
	settings.agent_port = AGENT_PORT;
	settings.odom_timer_delay = 100;
	settings.lidar_timer_delay = 100;
	settings.imu_timer_delay = 50;

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
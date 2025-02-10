#include "usb_controller/usb_controller.hpp"

void UsbController::clearInputBuffer() {
    while (Serial.available())
        Serial.read();
}

String UsbController::readInput() {
    String str;
    char c;

    clearInputBuffer();
    while (1)
    {
        while (!Serial.available()) vTaskDelay(1); // Ожидание ввода

        c = Serial.read();
        
        if (c == '\r')
            break;
        else if (c == 127){
            str.remove(str.length() - 1);
            Serial.print("\b \b");
            }
        else{
            Serial.print(c);
            str += c;
        }

        
        // Serial.print(Serial.read());
    }

    return str;
}

error_t UsbController::read_String(String *str, error_t validator(String) = nullptr) {
    String buffer = readInput();

    if(validator != nullptr)
    {
        error_t error = validator(buffer);
        if(error == 0) {
            buffer.trim();
            *str = buffer;
            return error;
        } else {
            Serial.println("\r\nОшибка: " + String(error));
            return error;
        }
    } else {
        buffer.trim();
        *str = buffer;
        return 0;
    }
}

error_t UsbController::read_uint32_t(uint32_t *num, error_t validator(String) = nullptr) {
    String buffer = readInput();

    if(validator != nullptr)
    {
        error_t error = validator(buffer);
        if(error == 0) {
            *num = buffer.toInt();
            return error;
        } else {
            Serial.println("\r\nОшибка: " + String(error));
            return error;
        }
    } else {
        *num = buffer.toInt();
        return 0;
    }
}

error_t UsbController::read_float(float *num, error_t validator(String) = nullptr) {
    String buffer = readInput();

    if(validator != nullptr)
    {
        error_t error = validator(buffer);
        if(error == 0) {
            *num = buffer.toFloat();
            return error;
        } else {
            Serial.println("\r\nОшибка: " + String(error));
            return error;
        }
    } else 
    {
        *num = buffer.toFloat();
        return 0;
    }
}

void UsbController::comMenu() {
	clearInputBuffer();
    Preferences pref_test;
    SettingUpdate update;
    uint32_t buffer_num;
    float buffer_float;
    float idle_consumption;
    float consumption_with_load;
    while (true) {
        // Вывод меню
        Serial.println("\r\nМеню:");
        Serial.println("1) Настройки");
        Serial.println("2) Вывести информацию о системе");
        Serial.println("5) Рестарт");
        Serial.println("6) Сброс(очистка памяти + рестарт)");
        Serial.println("0) Выйти");
        
        read_uint32_t(&buffer_num);
        switch (buffer_num) {
            case 1:
                settingsMenu();
                break;
            case 2:
                outputInfo();
                break;
            case 5:
                ESP.restart();
                break;
            case 6:
				nvs_flash_erase();
				nvs_flash_init();
                ESP.restart();
                break;
            default:
                Serial.print("\r\n");
				clearInputBuffer();
                return;
        }
    }
}

void UsbController::outputInfo() {
    Serial.printf("\r\nАккумулятор #%d\r\n", settings.turtle_id);

    Serial.printf("Текущий IP: %s\r\n", WiFi.localIP().toString().c_str());
    Serial.printf("Текущий уровень свободной памяти в куче:\t%d (bytes)\r\n", ESP.getFreeHeap());
}

void UsbController::settingsMenu() {
    DECLARE_SETTING_TYPES_VARIANT(UNIQUE_SETTINGS_TYPES) buffer;
    DECLARE_SETTING_TYPES_LINKS_VARIANT(UNIQUE_SETTINGS_TYPES) setting_field;

	SettingUpdate update;

    while (true) {
        // Вывод меню настроек
        Serial.println("\r\nМеню\\Настройки:");

        GEN_SETTINGS_OUTPUT_DEFAULT(setting_field, UNIQUE_SETTINGS_TYPES)
        Serial.println("0) Назад");

        GENERATE_SERIAL_INPUTS(settingUpdateQueue, update, SETTINGS_FIELDS)
    }
}

void UsbController::usbTask(void *pvParameters) {
    while (true) {
        if (Serial.available()) { // Позже сделаем возможность прерывать поток
            comMenu();
        }

        vTaskDelay(settings.usb_delay);
    }
}

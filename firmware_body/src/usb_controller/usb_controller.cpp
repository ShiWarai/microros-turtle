#include "usb_controller/usb_controller.hpp"

#define DECLARE_SERIAL_PRINT_ITER(TYPE, F1, F2, SETTING_POINTER) \
if(SETTINGS_INFO[i].type == #TYPE) \
{ \
    Serial.print(SETTINGS_INFO[i].name); \
    Serial.print("\r\t\t\t="); \
    Serial.print(*(std::get<TYPE*>(SETTING_POINTER))); \
}

#define GEN_SETTINGS_OUTPUT_DEFAULT(SETTING_POINTER, TYPES) \
for (unsigned short i = 0; i < SETTING_TYPE::SETTINGS_COUNT; i++) { \
    SETTING_POINTER = getSettingFieldPointer(i); \
    Serial.print(String(i+1)+") "); \
    TYPES(DECLARE_SERIAL_PRINT_ITER, SETTING_POINTER) \
    Serial.println(); \
}

#define GENERATE_UNIQUE_BUFFERS(TYPE, F1, F2, ...) \
TYPE buffer_##TYPE;

#define GENERATE_SERIAL_INPUT_CASE(TYPE, NAME, REBOOT_IS_REQUIRED, INPUT_VALIDATOR, UPDATE_QUEUE, UPDATE) \
case NAME: \
    Serial.printf("\r\nВведите новый %s: ", SETTINGS_INFO[NAME].name);\
    if(read_##TYPE(&buffer_##TYPE, SETTINGS_INFO[NAME].input_validator) != 0) { \
        Serial.println("\r\nОшибка ввода"); \
        break; \
    } \
    update.value = buffer_##TYPE; \
    update.key = NAME; \
    xQueueSend(UPDATE_QUEUE, &UPDATE, portMAX_DELAY); \
    if(SETTINGS_INFO[NAME].reboot_is_required) { \
        vTaskDelay(1000); \
        ESP.restart(); \
    } else \
        vTaskDelay(100); \
    break;

#define GENERATE_SERIAL_INPUTS(UPDATE_QUEUE, UPDATE, FILEDS) \
UNIQUE_SETTINGS_TYPES(GENERATE_UNIQUE_BUFFERS) \
read_uint32_t(&buffer_uint32_t); \
switch (buffer_uint32_t-1) { \
    FILEDS(GENERATE_SERIAL_INPUT_CASE, UPDATE_QUEUE, UPDATE) \
    default: \
        clearInputBuffer(); \
        return; \
}

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
    }

    return str;
}

// Название функции менять нельзя, чтобы работали макросы
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

// Название функции менять нельзя, чтобы работали макросы
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

// Название функции менять нельзя, чтобы работали макросы
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
        Serial.println("3) Рестарт");
        Serial.println("4) Очистка памяти");
        Serial.println("0) Выйти");
        
        read_uint32_t(&buffer_num);
        switch (buffer_num) {
            case 1:
                settingsMenu();
                break;
            case 2:
                printSystemInfo();
                break;
            case 3:
                ESP.restart();
                break;
            case 4:
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

void UsbController::printSystemInfo() {
    Serial.printf("\r\nРобот #%d\r\n", settings.turtle_id);

    Serial.printf("Текущий IP: %s\r\n", WiFi.localIP().toString().c_str());
    Serial.printf("Текущий уровень свободной памяти в куче:\t%d (bytes)\r\n", ESP.getFreeHeap());
}

void UsbController::settingsMenu() {
    DECLARE_SETTING_TYPES_VARIANT(UNIQUE_SETTINGS_TYPES) buffer;
    DECLARE_SETTING_TYPES_LINKS_VARIANT(UNIQUE_SETTINGS_TYPES) setting_field;

	SettingUpdate update;

    while (true) {
        // Вывод меню настроек
        Serial.println("\r\nНастройки:");

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

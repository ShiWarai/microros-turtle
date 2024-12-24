#pragma once

#include <Arduino.h>

struct SETTING_INFO {
    const char *name;
    const char *type;
    bool reboot_is_required;
    error_t (*input_validator)(String str);
};
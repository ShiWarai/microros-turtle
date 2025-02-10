#include "settings/preferences_validators.hpp"

error_t validate_id(String str) {
    int id = str.toInt();

    if(id > 0 && id < 256)
        return 0;
    else
        return 1;
}

error_t validate_uint(String str) {
    int id = str.toInt();

    if(id >= 0)
        if(id == 0 && !str.equals("0")) // На случай, если парсинг неудачный
            return 2;
        else
            return 0;
    else
        return 1; // Число со знаком
}
idf_component_register(
    SRCS "WiFi_AP_Server.cpp" "Sensoren.cpp" "I2SFullDuplex.cpp" "main.cpp" "ble_hid_keyboard.cpp" "Audio.cpp" "RGB.cpp" "TaskMonitor.cpp" "speaker.cpp" "DbSensor.cpp" "configLoader.c"
    INCLUDE_DIRS ""
    REQUIRES ld2412_local spiffs json arduino-esp32-3.2.1 AHTxx MTS4x OPT300x SparkFun_ENS160 espressif__esp-zigbee-lib esp_wifi nvs_flash
    PRIV_REQUIRES esp_http_server app_update
)
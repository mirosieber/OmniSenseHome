#ifndef RGB_H
#define RGB_H

#include "configLoader.h"
#include <Arduino.h>

void setRgbLedColor(app_config_t *config, uint8_t red, uint8_t green,
                    uint8_t blue);
// set brightness of RGB LED based on ambient light
void setRgbLedBrightness(app_config_t *config, uint8_t brightness);
// set RGB LED color and brightness based on ZigbeeLight state
void onRgbLightChange(app_config_t *config, bool state, uint8_t level);
// dimms RGB LED brightness up and down automatically
void toggleRgbBlink(app_config_t *config, bool on);
#endif
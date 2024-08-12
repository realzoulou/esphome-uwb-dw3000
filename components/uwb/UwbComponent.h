#pragma once

#include "esphome.h"
#include "esphome/core/component.h"

#include "Dw3000Device.h"
/*
#ifdef USE_ESP32
#include <WiFi.h>
#include "AsyncUDP.h"
#endif

#ifdef USE_ESP8266
#include <ESP8266WiFi.h>
#include "ESPAsyncUDP.h"
#endif
*/

using namespace esphome;

class UwbComponent : public esphome::Component {
public:
    UwbComponent();
    void setup();
    void dump_config();
    void loop();

private:
    const char* TAG = "uwb";
    Dw3000Device* mDevice = nullptr;
};

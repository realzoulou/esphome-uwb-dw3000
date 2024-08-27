#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#include "Dw3000Device.h"
#include "UwbListener.h"

namespace esphome {
namespace uwb {

typedef enum eUwbRole {
    UWB_ROLE_UNKNOWN = 0,
    UWB_ROLE_ANCHOR = 0x10,
    UWB_ROLE_TAG = 0x20,
} eUwbRole;


class UwbComponent : public esphome::Component, public UwbListener {

public:
    UwbComponent();
    void setup();
    void dump_config();
    void loop();
    inline float get_setup_priority() const override { return setup_priority::DATA; }
    inline void setRole(const eUwbRole role) { mRole = role; };
    inline void setDistanceSensor(sensor::Sensor *distanceSensor) { mDistanceSensor = distanceSensor; }

private:
    std::string roleToString(const eUwbRole role);
    // from UwbListener
    void onDistanceUpdated(double distanceMeters, uint32_t updateMillis);

    static const char* TAG;
    eUwbRole mRole{UWB_ROLE_UNKNOWN};
    Dw3000Device* mDevice{nullptr};
    sensor::Sensor* mDistanceSensor{nullptr};
};

}  // namespace uwb
}  // namespace esphome

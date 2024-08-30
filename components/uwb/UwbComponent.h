#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#include "Dw3000Device.h"

#include "UwbAnchorConfig.h"
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

    // from esphome::Component
    virtual void setup();
    virtual void dump_config();
    virtual void loop();
    virtual inline float get_setup_priority() const override { return setup_priority::DATA; }

    // UwbComponent specific
    inline void setDeviceId(const uint8_t id) { mDeviceId = id; }
    inline void setRole(const eUwbRole role) { mRole = role; };
    inline void setDistanceSensor(sensor::Sensor *distanceSensor) { mDistanceSensor = distanceSensor; }
    void addAnchor(const uint8_t id, const double latitude, const double longitude);

private:
    // from UwbListener
    void onDistanceUpdated(double distanceMeters, uint32_t updateMillis);

    static std::string roleToString(const eUwbRole role);

    static const char* TAG;
    eUwbRole mRole{UWB_ROLE_UNKNOWN};
    uint8_t mDeviceId{0};
    std::vector<std::shared_ptr<const UwbAnchorConfig>> mAnchorConfigs;
    Dw3000Device* mDevice{nullptr};
    sensor::Sensor* mDistanceSensor{nullptr};
};

}  // namespace uwb
}  // namespace esphome

#include <inttypes.h>

#include "esphome/core/log.h"

#include "UwbComponent.h"
#include "UwbAnchorDevice.h"
#include "UwbTagDevice.h"

namespace esphome {
namespace uwb {

const char* UwbComponent::TAG = "uwb";

UwbComponent::UwbComponent() {}

void UwbComponent::setup() {
    switch (mRole) {
        case UWB_ROLE_ANCHOR:
            mDevice = new UwbAnchorDevice();
            break;
        case UWB_ROLE_TAG:
            mDevice = new UwbTagDevice(mAnchorConfigs);
            mAnchorConfigs.clear(); // no longer needed
            break;
        default:
            ESP_LOGE(TAG, "unknown role %i", mRole);
            break;
    }
    if (mDevice != nullptr) {
        mDevice->setDeviceId(mDeviceId);
        mDevice->setListener(this);
        mDevice->setup();
    }
}

void UwbComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "uwb");
    ESP_LOGCONFIG(TAG, " role: %s", roleToString(mRole).c_str());
}

void UwbComponent::loop() {
    mDevice->loop();
}

void UwbComponent::addAnchor(const uint8_t id, const double latitude, const double longitude) {
    const auto anchor = std::make_shared<const UwbAnchorConfig>(id, latitude, longitude);
    mAnchorConfigs.push_back(anchor);
}

void UwbComponent::onDistanceUpdated(double distanceMeters, uint32_t updateMillis) {
    if (mDistanceSensor != nullptr) {
        mDistanceSensor->publish_state(distanceMeters);
    }
}

std::string UwbComponent::roleToString(const eUwbRole role) {
    switch(role) {
        case UWB_ROLE_ANCHOR:
            return "anchor";
        case UWB_ROLE_TAG:
            return "tag";
        case UWB_ROLE_UNKNOWN:
            return "unknown";
        default:
            return "???";
    }
}

}  // namespace uwb
}  // namespace esphome

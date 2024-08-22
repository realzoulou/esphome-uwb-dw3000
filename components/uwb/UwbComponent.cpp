#include "esphome/core/log.h"

#include "UwbComponent.h"
#include "UwbAnchorControllerDevice.h"
#include "UwbAnchorPeripheralDevice.h"
#include "UwbTagDevice.h"

namespace esphome {
namespace uwb {

const char* UwbComponent::TAG = "uwb";

UwbComponent::UwbComponent() {}

void UwbComponent::setup() {
    switch (mRole) {
        case UWB_ROLE_ANCHOR_CONTROLLER:
            mDevice = new UwbAnchorControllerDevice();
            break;
        case UWB_ROLE_ANCHOR_PERIPHERAL:
            mDevice = new UwbAnchorPeripheralDevice();
            break;
        case UWB_ROLE_TAG:
            mDevice = new UwbTagDevice();
            break;
        default:
            ESP_LOGE(TAG, "unknown role %i", mRole);
            break;
    }
    if (mDevice != nullptr) {
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

std::string UwbComponent::roleToString(const eUwbRole role) {
    switch(role) {
        case UWB_ROLE_ANCHOR_CONTROLLER:
            return "anchor_controller";
        case UWB_ROLE_ANCHOR_PERIPHERAL:
            return "anchor_peripheral";
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

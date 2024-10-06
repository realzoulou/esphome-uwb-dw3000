#include <utility>

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
        {
            const sensor::Sensor *distSensor;
            const auto search = mDistanceSensors.find(mDeviceId);
            if (search != mDistanceSensors.cend()) {
                distSensor = search->second;
            } else {
                distSensor = nullptr;
            }
            mDevice = new UwbAnchorDevice(mAnchorLatitude, mAnchorLongitude, mLatitudeSensor, mLongitudeSensor, distSensor);
        }
        break;
        case UWB_ROLE_TAG:
        {
            // assign distance sensor to Anchors, if configured in YAML
            for (const auto sensor : mDistanceSensors) {
                for (auto anchorConfig : mAnchors) {
                    if (sensor.first /* target device ID */ == anchorConfig->getId()) {
                        anchorConfig->setSensor(/* Sensor* */ sensor.second);
                    }
                }
            }
            mDevice = new UwbTagDevice(mAnchors, mRangingIntervalMs, mMaxAgeAnchorDistanceMs,
                                       mLatitudeSensor, mLongitudeSensor, mLocationErrorEstimateSensor);
        }
        break;
        default:
            ESP_LOGE(TAG, "unknown role %i", mRole);
            break;
    }
    // Anchors and Sensors no longer needed
    mAnchors.clear();
    mDistanceSensors.clear();

    if (mDevice != nullptr) {
        mDevice->setDeviceId(mDeviceId);
        mDevice->setLedsOffAfter(mLedsOffAfterMs);
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

void UwbComponent::addAnchor(const uint8_t id, const double latitude, const double longitude,
                             const double minDistance, const double maxSpeed) {
    auto anchor = std::make_shared<UwbAnchorData>(id, latitude, longitude, minDistance, maxSpeed);
    mAnchors.push_back(anchor);
}

void UwbComponent::addDistanceSensor(const uint8_t targetDeviceId, const sensor::Sensor* sensor) {
    auto pair = std::make_pair(targetDeviceId, sensor);
    mDistanceSensors.insert(pair);
}

void UwbComponent::addLatitudeSensor(const sensor::Sensor* sensor) {
    mLatitudeSensor = const_cast<sensor::Sensor*>(sensor);
}

void UwbComponent::addLongitudeSensor(const sensor::Sensor* sensor) {
    mLongitudeSensor = const_cast<sensor::Sensor*>(sensor);
}

void UwbComponent::addErrorEstimateSensor(const sensor::Sensor* sensor) {
    mLocationErrorEstimateSensor = const_cast<sensor::Sensor*>(sensor);
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

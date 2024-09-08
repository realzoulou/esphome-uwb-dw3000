#include "UwbAnchorData.h"

namespace esphome {
namespace uwb {

void UwbAnchorData::setDistance(double distanceMeters) {
    mDistanceToTag = distanceMeters;
    mMillisDistanceToTag = millis();
    if (mSensor != nullptr) {
        auto sensor = const_cast<sensor::Sensor*>(mSensor); // remove 'const' in order to publish_state
        sensor->publish_state(mDistanceToTag);
    }
}

double UwbAnchorData::getDistance(uint32_t* millis) const {
    if (millis != nullptr) {
        *millis = mMillisDistanceToTag;
    }
    return mDistanceToTag;
}

}  // namespace uwb
}  // namespace esphome

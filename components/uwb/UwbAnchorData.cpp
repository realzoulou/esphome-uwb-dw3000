#include "UwbAnchorData.h"

#include <cmath>

namespace esphome {
namespace uwb {

const char* UwbAnchorData::TAG = "tag"; // running in role=tag context

void UwbAnchorData::setDistance(double distanceMeters) {
    const uint32_t now = millis();
    /* diff in [m] to previous distance. */
    const double diffToPrevDistance = distanceMeters - mDistanceToTag;

    /* speed in [m/s] that the tag seems to have changed its location */
    const double speed = distanceMeters /
        ((double)(now - mMillisDistanceToTag + 500) / 1000.0);

    /* Avoid outliers. */
    if (std::fabs(diffToPrevDistance) >= MIN_DISTANCE) {
        if (std::fabs(speed) <= MAX_SPEED) {
            mDistanceToTag = distanceMeters;
            mMillisDistanceToTag = millis();
            if (mSensor != nullptr) {
                auto sensor = const_cast<sensor::Sensor*>(mSensor); // remove 'const' in order to publish_state
                sensor->publish_state(mDistanceToTag);
            }
        } else {
            ESP_LOGW(TAG, "anchor 0x%02x: dist %.2fm results in speed %.2fm/s > threshold %.2fm, keep %.2f",
                mId, distanceMeters, speed, MAX_SPEED, mDistanceToTag);
        }
    } else {
        // not a warning because this happens very frequently and is actually normal
        ESP_LOGI(TAG, "anchor 0x%02x: dist %.2fm (%.2fm) < threshold %.2fm, keep %.2f",
            mId, distanceMeters, diffToPrevDistance, MIN_DISTANCE, mDistanceToTag);
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

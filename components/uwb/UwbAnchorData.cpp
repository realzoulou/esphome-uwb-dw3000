#include "UwbAnchorData.h"

#include <cmath>

namespace esphome {
namespace uwb {

const char* UwbAnchorData::TAG = "tag"; // running in role=tag context

void UwbAnchorData::setDistance(double distanceMeters) {
    const uint32_t now = millis();

    if (std::isnan(distanceMeters)) {
        if (mSensor != nullptr) {
            auto sensor = const_cast<sensor::Sensor*>(mSensor); // remove 'const' in order to publish_state
            // don't update mMillisDistanceToTag to now
            mDistanceToTag = NAN;
            sensor->publish_state(NAN);
        }
        ESP_LOGW(TAG, "anchor 0x%02X away", mId);
        return;
    }

    double diffToPrevDistance, diffMillis, speed;
    if (! std::isnan(mDistanceToTag)) {
        /* diff in [m] to previous distance. */
        diffToPrevDistance = distanceMeters - mDistanceToTag;
        /* diff in [ms] to previous millis() when distance was calculated. */
        diffMillis = (double) (now - mMillisDistanceToTag);
        /* speed in [m/s] that the tag seems to have changed its distance. */
        speed = (diffToPrevDistance * 1000.0) / diffMillis;
    } else {
        // set values such that outlier detection is passed.
        diffToPrevDistance = MIN_DISTANCE;
        speed = MAX_SPEED;
        diffMillis = NAN;
        if (!std::isnan(distanceMeters)) {
            // changed from 'away' to 'back'
            ESP_LOGW(TAG, "anchor 0x%02X back", mId);
        }
    }
    
    /* Avoid outliers. */
    if (std::fabs(diffToPrevDistance) >= MIN_DISTANCE) {
        if (std::fabs(speed) <= MAX_SPEED) {
            mDistanceToTag = distanceMeters;
            mMillisDistanceToTag = now;
            if (mSensor != nullptr) {
                auto sensor = const_cast<sensor::Sensor*>(mSensor); // remove 'const' in order to publish_state
                sensor->publish_state(mDistanceToTag);
            }
        } else {
            ESP_LOGW(TAG, "anchor 0x%02X: dist %.2fm (Δ%.2fm in %.0fms) speed %.2fm/s > threshold %.2fm/s, keep %.2fm",
                mId, distanceMeters, diffToPrevDistance, diffMillis, speed, MAX_SPEED, mDistanceToTag);
        }
    } else {
        // update timestamp nevertheless to avoid 'away' detection kicks in
        mMillisDistanceToTag = now;
        // update also distance otherwise 'over-speed' detection could kick in
        mDistanceToTag = distanceMeters;
        // not a warning because this happens very frequently and is actually normal
        ESP_LOGI(TAG, "anchor 0x%02X: dist %.2fm (prev %.2fm) < threshold %.2fm, keep %.2fm",
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

/*
 * Copyright 2024 realzoulou
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "UwbAnchorData.h"

#include <cmath>

namespace esphome {
namespace uwb {

const char* UwbAnchorData::TAG = "tag"; // running in role=tag context

void UwbAnchorData::setDistance(double distanceMeters, double distErrorEstimate, bool enforce) {
    const uint32_t now = millis();

    if (enforce) {
        // enforced setting of distance
        mMillisDistanceToTag = now;
        mDistanceToTag = distanceMeters;
        mDistanceToTagErrorEstimate = distErrorEstimate;
        if (mSensor != nullptr) {
            mSensor->publish_state(mDistanceToTag);
        }
        return;
    }

    if (std::isnan(distanceMeters)) {
        // don't update mMillisDistanceToTag to now
        mDistanceToTag = NAN;
        mDistanceToTagErrorEstimate = NAN;
        if (mSensor != nullptr) {
            mSensor->publish_state(NAN);
        }
        std::ostringstream msg;
        msg << "anchor 0x" << std::hex << +mId << std::dec << " away";
        MSG_LOGW(msg);
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
            std::ostringstream msg;
            msg << "anchor 0x" << std::hex << +mId << std::dec << " back";
            MSG_LOGW(msg);
        }
    }

    /* Avoid outliers. */
    if (std::fabs(diffToPrevDistance) >= MIN_DISTANCE) {
        if (std::fabs(speed) <= MAX_SPEED) {
            mDistanceToTag = distanceMeters;
            mDistanceToTagErrorEstimate = distErrorEstimate;
            mMillisDistanceToTag = now;
            if (mSensor != nullptr) {
                mSensor->publish_state(mDistanceToTag);
            }
        } else {
            std::ostringstream msg;
            msg << "anchor 0x" << std::hex << +mId << std::dec
                << ": dist " << std::fixed << std::setprecision(2) << +distanceMeters << "m (Δ"
                << +diffToPrevDistance << "m in " << std::setprecision(0) << +diffMillis << "ms) speed "
                << std::setprecision(2) << +speed << "m/s > threshold " << +MAX_SPEED << "m/s, keep "
                << +mDistanceToTag << "m";
            MSG_LOGW(msg);
        }
    } else {
        /* Minimal change in distance to tag happened. Just store. */
        mMillisDistanceToTag = now;
        mDistanceToTag = distanceMeters;
        mDistanceToTagErrorEstimate = distErrorEstimate;
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

#pragma once

#include <cinttypes>

#ifndef __UT_TEST__
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/hal.h"
#else
namespace sensor { class Sensor; }
#endif

namespace esphome {
namespace uwb {

class UwbAnchorData {

public:
    UwbAnchorData(const uint8_t id, const double latitude, const double longitude,
                  const double minDistance, const double maxSpeed)
    : mId(id), mLatitude(latitude), mLongitude(longitude), MIN_DISTANCE(minDistance), MAX_SPEED(maxSpeed) {}

    inline void setSensor(const sensor::Sensor* sensor) { mSensor = sensor; }
    inline const sensor::Sensor* getSensor() const { return mSensor; }

    inline uint8_t getId() const { return mId; }
    inline double getLatitude() const { return mLatitude; }
    inline double getLongitude() const { return mLongitude; }

    void setDistance(double distanceMeters);
    double getDistance(uint32_t* millis) const;

private:
    static const char* TAG;

    /* ID of the anchor. */
    const uint8_t mId;

    /* GNSS latitude of the anchor. */
    const double mLatitude;

    /* GNSS longitude of the anchor. */
    const double mLongitude;

    /* Distance sensor. */
    const sensor::Sensor* mSensor;

    /* Minimum absolute difference of a new distance to be reported. */
    const double MIN_DISTANCE;

    /* maximum speed in [m/s] that a tag can change its location typically. */
    const double MAX_SPEED;

    /* Latest Distance in [m] to tag. */
    double mDistanceToTag{0.0};

    /* Latest millis() when distance was set. */
    uint32_t mMillisDistanceToTag{0};
};

}  // namespace uwb
}  // namespace esphome

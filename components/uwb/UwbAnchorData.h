#pragma once

#include <cinttypes>
#include <cmath>
#include <iomanip>
#include <sstream>

#ifdef ESP32
    #include "esphome/components/sensor/sensor.h"
    #include "esphome/core/hal.h"
    #define MSG_LOGE(ostringstream) ESP_LOGE(TAG, "%s", ostringstream.str().c_str())
    #define MSG_LOGW(ostringstream) ESP_LOGW(TAG, "%s", ostringstream.str().c_str())
    #define MSG_LOGI(ostringstream) ESP_LOGI(TAG, "%s", ostringstream.str().c_str())
    #define MSG_LOGV(ostringstream) ESP_LOGV(TAG, "%s", ostringstream.str().c_str())
#else
    namespace sensor {
        class Sensor {
            public:
            void publish_state(double) {}
        };
    }
    #include <iostream>
    #define MSG_LOGE(ostringstream) std::cerr << TAG << ": " << ostringstream.str() << std::endl
    #define MSG_LOGW(ostringstream) std::clog << TAG << ": " << ostringstream.str() << std::endl
    #define MSG_LOGI(ostringstream) std::cout << TAG << ": " << ostringstream.str() << std::endl
    #define MSG_LOGV(ostringstream) std::cout << TAG << ": " << ostringstream.str() << std::endl
    #define millis() (4711U)
#endif

namespace esphome {
namespace uwb {

class UwbAnchorData {

public:
    UwbAnchorData(const uint8_t id, const double latitude, const double longitude,
                  const double minDistance, const double maxSpeed)
    : mId(id), mLatitude(latitude), mLongitude(longitude), MIN_DISTANCE(minDistance), MAX_SPEED(maxSpeed) {}

    inline void setSensor(sensor::Sensor* sensor) { mSensor = sensor; }
    inline sensor::Sensor* getSensor() const { return mSensor; }

    inline uint8_t getId() const { return mId; }
    inline double getLatitude() const { return mLatitude; }
    inline double getLongitude() const { return mLongitude; }

    void setDistance(double distanceMeters, double distanceErrorEstimate = NAN, bool enforce = false);
    double getDistance(uint32_t* millis) const;
    inline double getDistanceErrorEstimate() const { return mDistanceToTagErrorEstimate; }

private:
    static const char* TAG;

    /* ID of the anchor. */
    const uint8_t mId;

    /* GNSS latitude of the anchor. */
    const double mLatitude;

    /* GNSS longitude of the anchor. */
    const double mLongitude;

    /* Distance sensor. */
    sensor::Sensor* mSensor{nullptr};

    /* Minimum absolute difference of a new distance to be reported. */
    const double MIN_DISTANCE;

    /* Maximum speed in [m/s] that a tag can change its location typically. */
    const double MAX_SPEED;

    /* Distance in [m] to tag. */
    double mDistanceToTag{NAN};

    /* Distance error estimate in [m] to tag. */
    double mDistanceToTagErrorEstimate{NAN};

    /* millis() when distance was set. */
    uint32_t mMillisDistanceToTag{0};
};

}  // namespace uwb
}  // namespace esphome

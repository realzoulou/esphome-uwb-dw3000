#pragma once

#include <cstdint>

namespace esphome {
namespace uwb {

class UwbAnchorConfig {

public:
    UwbAnchorConfig(const uint8_t id, const double atitude, const double longitude)
    : mId(id), mLatitude(mLatitude), mLongitude(longitude) {}

    uint8_t getId() const { return mId; }
    double getLatitude() const { return mLatitude; }
    double getLongitude() const { return mLongitude; }

private:
    /* ID of the anchor. */
    const uint8_t mId;

    /* GNSS latitude of the anchor. */
    const double mLatitude;

    /* GNSS longitude of the anchor. */
    const double mLongitude;
};

}  // namespace uwb
}  // namespace esphome

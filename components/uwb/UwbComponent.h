#pragma once

#include <inttypes.h>
#include <map>
#include <utility>

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#include "Dw3000Device.h"

#include "UwbAnchorData.h"
#include "UwbListener.h"

namespace esphome {
namespace uwb {

typedef enum eUwbRole {
    UWB_ROLE_UNKNOWN = 0,
    UWB_ROLE_ANCHOR = 0x10,
    UWB_ROLE_TAG = 0x20,
} eUwbRole;

/* Default minimum absolute difference in [m] of a new distance to be reported.
   Avoids to report small changes in distance (and location) in every ranging cycle.
*/
const static double MIN_DISTANCE_CHANGE_DEFAULT = 0.05; /* 5cm */

/* Default maximum speed in [m/s] that a tag can change its location typically. */
const static double MAX_SPEED_DEFAULT           = 1.0;  /* 1 m/s */

/* Default ranging interval time in [ms]. */
const static uint32_t RANGING_INTERVAL_TIME     = 1000; /* every 1s */

class UwbComponent : public esphome::Component {

public:
    UwbComponent();

    // from esphome::Component
    virtual void setup();
    virtual void dump_config();
    virtual void loop();
    virtual inline float get_setup_priority() const override { return setup_priority::DATA; }

    // UwbComponent specific
    inline void setDeviceId(const uint8_t id) { mDeviceId = id; }
    inline void setRole(const eUwbRole role) { mRole = role; };
    inline void setRangingInterval(const uint32_t rangingIntervalMs) { mRangingIntervalMs = rangingIntervalMs; }
    void addAnchor(
        // mandatory parameters
        const uint8_t id, const double latitude, const double longitude,
        // optional parameters
        const double minDistance = MIN_DISTANCE_CHANGE_DEFAULT,
        const double maxSpeed = MAX_SPEED_DEFAULT
    );
    void addDistanceSensor(const uint8_t targetDeviceId, const sensor::Sensor* sensor);

private:
    static std::string roleToString(const eUwbRole role);

    static const char* TAG;
    eUwbRole mRole{UWB_ROLE_UNKNOWN};
    uint8_t mDeviceId{0};
    uint32_t mRangingIntervalMs{RANGING_INTERVAL_TIME};
    std::vector<std::shared_ptr<UwbAnchorData>> mAnchors;
    std::map<const uint8_t, const sensor::Sensor*> mDistanceSensors;
    Dw3000Device* mDevice{nullptr};
};

}  // namespace uwb
}  // namespace esphome

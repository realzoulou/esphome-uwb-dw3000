#pragma once

#include <cstdint>

#include "UwbListener.h"

namespace esphome {
namespace uwb {

class Dw3000Device {
public:
    Dw3000Device();

    virtual void setup();
    virtual void loop();

    /* get last distance in [m] and time in [ms] when last updated */
    virtual double getLastDistance(uint32_t* timeMillis) const;
    /* set last distance in [m] */
    virtual void setLastDistance(const double distance);

    inline void setListener(UwbListener* listener) { mListener = listener; }

    static uint8_t getNextTxSequenceNumberAndIncrease();

protected:
    static const char* TAG;

    /* Total amount of TX errors.*/
    uint32_t mTxErrorCount{0};

    /* Last measured distance. */
    double mLastDistance{0.0};
    /* millis() of when last distance was updated. */
    uint32_t mLastDistanceUpdatedMs{0};

private:
    static uint8_t txSequenceNumber;

    UwbListener* mListener{nullptr};
};

}  // namespace uwb
}  // namespace esphome

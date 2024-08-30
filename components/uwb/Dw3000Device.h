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

    inline void setDeviceId(const uint8_t id) { mDeviceId = id; }
    inline uint8_t getDeviceId() const { return mDeviceId; }

    inline void setListener(UwbListener* listener) { mListener = listener; }

    static uint8_t getNextTxSequenceNumberAndIncrease();

protected:
    static const char* TAG;

    /* ID of this device. */
    uint8_t mDeviceId{0};

    /* Total amount of TX errors.*/
    uint32_t mTxErrorCount{0};

    UwbListener* mListener{nullptr};

private:
    static uint8_t txSequenceNumber;
};

}  // namespace uwb
}  // namespace esphome

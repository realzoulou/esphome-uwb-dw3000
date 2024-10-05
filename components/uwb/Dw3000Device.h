#pragma once

#include <cstdint>

#include "UwbListener.h"

#include "esphome/core/helpers.h"

#include "dw3000_device_api.h"

namespace esphome {
namespace uwb {

/* Default antenna delay values for 64 MHz PRF */
#define TX_ANT_DLY  (16385) // Decawave default was confirmed to be good (by ranging with 8.0m distance)
#define RX_ANT_DLY  TX_ANT_DLY // for simplicity: RX = TX antenna delay

/* Macros for surrounding a code block which is time-critical and should not be interrupted. */
#define TIME_CRITICAL_START()
#define TIME_CRITICAL_END()

class Dw3000Device {
public:
    Dw3000Device();

    virtual void setup();
    virtual void loop();

    inline void setDeviceId(const uint8_t id) { mDeviceId = id; }
    inline uint8_t getDeviceId() const { return mDeviceId; }

    inline void setLedsOffAfter(const uint32_t ledsOffAfterMs) { mLedsOffAfterMs = ledsOffAfterMs; }
    inline uint32_t getLedsOffAfter() const { return mLedsOffAfterMs; }
    virtual void maybeTurnLedsOff();

    static uint8_t getNextTxSequenceNumberAndIncrease();
    static dwt_config_t* getConfig();

protected:
    static const char* TAG;

    /* ID of this device. */
    uint8_t mDeviceId{0};

    /* Total amount of TX errors.*/
    uint32_t mTxErrorCount{0};

    /* Time in [ms] after startup to turn LEDs off, 0 keeps LEDs on). */
    uint32_t mLedsOffAfterMs{0};
    /* Remembers that maybeTurnLedsOff() has already done its job. */
    bool mLedsCheckDone{false};

    HighFrequencyLoopRequester mHighFreqLoopRequester;
private:
    static uint8_t txSequenceNumber;
};

}  // namespace uwb
}  // namespace esphome

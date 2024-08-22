#pragma once

#include <cstdint>

namespace esphome {
namespace uwb {

class Dw3000Device {
public:
    Dw3000Device();

    virtual void setup();
    virtual void loop();

    static uint8_t getNextTxSequenceNumberAndIncrease();

protected:
    static const char* TAG;

    /* Total amount of TX errors.*/
    uint32_t mTxErrorCount{0};

private:
    static uint8_t txSequenceNumber;
};

}  // namespace uwb
}  // namespace esphome

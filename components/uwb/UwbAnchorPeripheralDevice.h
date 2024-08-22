#pragma once

#include "UwbAnchorDevice.h"

namespace esphome {
namespace uwb {

class UwbAnchorPeripheralDevice : public UwbAnchorDevice {
public:
    UwbAnchorPeripheralDevice();

    virtual void setup();
    virtual void loop();

protected:
    static const char* TAG;
};

}  // namespace uwb
}  // namespace esphome
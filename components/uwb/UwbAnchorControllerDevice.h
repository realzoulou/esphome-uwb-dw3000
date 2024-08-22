#pragma once

#include "UwbAnchorDevice.h"

namespace esphome {
namespace uwb {

class UwbAnchorControllerDevice : public UwbAnchorDevice {
public:
    UwbAnchorControllerDevice();

    virtual void setup();
    virtual void loop();

protected:
    static const char* TAG;
};

}  // namespace uwb
}  // namespace esphome
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
    const char* TAG = "anchorC";
};

}  // namespace uwb
}  // namespace esphome
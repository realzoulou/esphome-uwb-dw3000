#pragma once

#include "Dw3000Device.h"

namespace esphome {
namespace uwb {

class UwbAnchorDevice : public Dw3000Device {
public:
    UwbAnchorDevice();

    virtual void setup();
    virtual void loop();

protected:
    const char* TAG = "anchor";
};

}  // namespace uwb
}  // namespace esphome
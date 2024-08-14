#pragma once

#include "Dw3000Device.h"

namespace esphome {
namespace uwb {

class UwbTagDevice : public Dw3000Device {
public:
    UwbTagDevice();

    virtual void setup();
    virtual void loop();

protected:
    const char* TAG = "tag";
};

}  // namespace uwb
}  // namespace esphome
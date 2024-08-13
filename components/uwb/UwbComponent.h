#pragma once

#include "esphome/core/component.h"

#include "Dw3000Device.h"

namespace esphome {
namespace uwb {

class UwbComponent : public esphome::Component {
public:
    UwbComponent();
    void setup();
    void dump_config();
    void loop();
    float get_setup_priority() const override { return setup_priority::DATA; }

protected:
    const char* TAG = "uwb";
    Dw3000Device* mDevice;
};

}  // namespace uwb
}  // namespace esphome

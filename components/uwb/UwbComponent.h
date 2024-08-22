#pragma once

#include "esphome/core/component.h"

#include "Dw3000Device.h"

namespace esphome {
namespace uwb {

typedef enum eUwbRole {
    UWB_ROLE_UNKNOWN = 0,
    UWB_ROLE_ANCHOR_CONTROLLER = 0x10,
    UWB_ROLE_ANCHOR_PERIPHERAL = 0x11,
    UWB_ROLE_TAG = 0x20,
} eUwbRole;

class UwbComponent : public esphome::Component {

public:
    UwbComponent();
    void setup();
    void dump_config();
    void loop();
    float get_setup_priority() const override { return setup_priority::DATA; }
    void setRole(const eUwbRole role) { mRole = role; };
    std::string roleToString(const eUwbRole role);

protected:
    static const char* TAG;
    eUwbRole mRole{UWB_ROLE_UNKNOWN};
    Dw3000Device* mDevice{nullptr};
};

}  // namespace uwb
}  // namespace esphome

#pragma once

namespace esphome {
namespace uwb {

class Dw3000Device {
public:
    Dw3000Device();

    virtual void setup();
    virtual void loop();

protected:
    const char* TAG = "Dw3000Device";
};

}  // namespace uwb
}  // namespace esphome

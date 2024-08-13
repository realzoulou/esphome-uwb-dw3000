#pragma once

namespace esphome {
namespace uwb {

class Dw3000Device {
public:
    Dw3000Device();

    void setup();
    void loop();

private:
    const char* TAG = "Dw3000Device";
};

}  // namespace uwb
}  // namespace esphome

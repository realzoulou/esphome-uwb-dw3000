#include "UwbTagDevice.h"

namespace esphome {
namespace uwb {

UwbTagDevice::UwbTagDevice() {}

void UwbTagDevice::setup() {
    Dw3000Device::setup();
}

void UwbTagDevice::loop() {
    Dw3000Device::loop();
}

}  // namespace uwb
}  // namespace esphome
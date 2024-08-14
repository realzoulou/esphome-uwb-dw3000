#include "UwbAnchorDevice.h"

namespace esphome {
namespace uwb {

UwbAnchorDevice::UwbAnchorDevice() {}

void UwbAnchorDevice::setup() {
    Dw3000Device::setup();
}

void UwbAnchorDevice::loop() {
    Dw3000Device::loop();
}

}  // namespace uwb
}  // namespace esphome
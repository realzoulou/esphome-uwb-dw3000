#include "UwbAnchorControllerDevice.h"

namespace esphome {
namespace uwb {

const char* UwbAnchorControllerDevice::TAG = "anchorC";

UwbAnchorControllerDevice::UwbAnchorControllerDevice() {}

void UwbAnchorControllerDevice::setup() {
    UwbAnchorDevice::setup();
}

void UwbAnchorControllerDevice::loop() {
    UwbAnchorDevice::loop();
}

}  // namespace uwb
}  // namespace esphome
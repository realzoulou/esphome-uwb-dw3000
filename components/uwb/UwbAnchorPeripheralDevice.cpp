#include "UwbAnchorPeripheralDevice.h"

namespace esphome {
namespace uwb {

const char* UwbAnchorPeripheralDevice::TAG = "anchorP";

UwbAnchorPeripheralDevice::UwbAnchorPeripheralDevice() {}

void UwbAnchorPeripheralDevice::setup() {
    UwbAnchorDevice::setup();
}

void UwbAnchorPeripheralDevice::loop() {
    UwbAnchorDevice::loop();
}

}  // namespace uwb
}  // namespace esphome
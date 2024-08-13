#include "esphome/core/log.h"

#include "UwbComponent.h"

namespace esphome {
namespace uwb {

UwbComponent::UwbComponent() {
    mDevice = new Dw3000Device();
}

void UwbComponent::setup() {
    ESP_LOGD(TAG, "setup");
    mDevice->setup();
}

void UwbComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "uwb");
}

void UwbComponent::loop() {
    mDevice->loop();
}

}  // namespace uwb
}  // namespace esphome

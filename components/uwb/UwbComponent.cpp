#include "esphome/core/log.h"

#include "UwbComponent.h"

using namespace esphome;

UwbComponent::UwbComponent() {

}

void UwbComponent::setup() {
    ESP_LOGI(TAG, "setup");
}

void UwbComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "uwb");
}

void UwbComponent::loop() {

}

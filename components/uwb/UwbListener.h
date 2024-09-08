#pragma once

#include <cstdint>

class UwbListener {
public:
    /* new distance in [m] calculated by source device to target device at a certain millis() */
    virtual void onDistanceUpdated(uint8_t sourceDeviceId, uint8_t targetDeviceId, double distanceMeters, uint32_t updateMillis) = 0;
};

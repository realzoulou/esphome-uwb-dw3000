#pragma once

#include <cstdint>

class UwbListener {
public:
    virtual void onDistanceUpdated(double distanceMeters, uint32_t updateMillis) = 0;
};

/*
 * Copyright 2024 realzoulou
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cstdint>

class UwbListener {
public:
    /* new distance in [m] calculated by source device to target device at a certain millis() */
    virtual void onDistanceUpdated(uint8_t sourceDeviceId, uint8_t targetDeviceId, double distanceMeters, uint32_t updateMillis) = 0;
};

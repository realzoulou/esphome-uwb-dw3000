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

#include "AntDelayCalibDeviceSelect.h"

namespace esphome {
namespace uwb {

void AntDelayCalibDeviceSelect::setCallback(AntDelayCalibDeviceSelectCallback* cb) {
    mDeviceSelectCallback = cb;
}

void AntDelayCalibDeviceSelect::control(const std::string &value) {
    if (mDeviceSelectCallback != nullptr) {
        mDeviceSelectCallback->controlAntennaDelayCalibrationDevice(value);
    }
}

}  // namespace uwb
}  // namespace esphome

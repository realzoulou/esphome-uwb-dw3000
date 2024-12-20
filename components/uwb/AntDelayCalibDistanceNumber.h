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

#include "esphome/components/number/number.h"

namespace esphome {
namespace uwb {

class AntDelayCalibDistanceNumberCallback {
    public:
        virtual void controlAntennaDelayCalibrationDistance(float value) = 0;
};

class AntDelayCalibDistanceNumber : public number::Number
{
    public:
        AntDelayCalibDistanceNumber() = default;

        /* set a callback when control() was called */
        void setCallback(AntDelayCalibDistanceNumberCallback* cb);

    protected: // from number::Number
        void control(float value) override;

    private:
        AntDelayCalibDistanceNumberCallback* mDistanceCallback{nullptr};
};

}  // namespace uwb
}  // namespace esphome

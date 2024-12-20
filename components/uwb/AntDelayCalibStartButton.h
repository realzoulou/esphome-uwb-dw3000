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

#include "esphome/components/button/button.h"

namespace esphome {
namespace uwb {

class AntDelayCalibStartCallback {
    public:
        virtual void pressedStartAntennaCalibration() = 0;
};

class AntDelayCalibStartButton : public button::Button
{
    public:
        AntDelayCalibStartButton() = default;

        /* set a callback when press_action() was called */
        void setCallback(AntDelayCalibStartCallback* cb);

    protected: // from button::button
        void press_action() override;

    private:
        AntDelayCalibStartCallback* mStartCallback{nullptr};
};

}  // namespace uwb
}  // namespace esphome

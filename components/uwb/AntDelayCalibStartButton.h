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

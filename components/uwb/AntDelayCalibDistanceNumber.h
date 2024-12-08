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

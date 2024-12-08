#pragma once

#include "esphome/components/select/select.h"

namespace esphome {
namespace uwb {

class AntDelayCalibDeviceSelectCallback {
    public:
        virtual void controlAntennaDelayCalibrationDevice(const std::string &value) = 0;
};

class AntDelayCalibDeviceSelect : public select::Select
{
    public:
        AntDelayCalibDeviceSelect() = default;

        /* set a callback when control() was called */
        void setCallback(AntDelayCalibDeviceSelectCallback* cb);

    protected: // from select::Select
        void control(const std::string &value) override;

    private:
        AntDelayCalibDeviceSelectCallback* mDeviceSelectCallback{nullptr};
};

}  // namespace uwb
}  // namespace esphome

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

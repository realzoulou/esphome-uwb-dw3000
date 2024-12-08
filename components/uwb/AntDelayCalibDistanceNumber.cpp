#include "AntDelayCalibDistanceNumber.h"

namespace esphome {
namespace uwb {

void AntDelayCalibDistanceNumber::setCallback(AntDelayCalibDistanceNumberCallback* cb) {
    mDistanceCallback = cb;
}

void AntDelayCalibDistanceNumber::control(float value) {
    if (mDistanceCallback != nullptr) {
        mDistanceCallback->controlAntennaDelayCalibrationDistance(value);
    }
}

}  // namespace uwb
}  // namespace esphome

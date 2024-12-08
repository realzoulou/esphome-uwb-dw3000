#include "AntDelayCalibStartButton.h"

namespace esphome {
namespace uwb {

void AntDelayCalibStartButton::setCallback(AntDelayCalibStartCallback* cb) {
    mStartCallback = cb;
}

void AntDelayCalibStartButton::press_action() {
    if (mStartCallback != nullptr) {
        mStartCallback->pressedStartAntennaCalibration();
    }
}

}  // namespace uwb
}  // namespace esphome

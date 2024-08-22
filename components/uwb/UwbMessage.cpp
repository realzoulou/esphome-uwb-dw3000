#include "UwbMessage.h"

#include <cstddef>

namespace esphome {
namespace uwb {

const char* UwbMessage::TAG = "UwbMessage";

UwbMessage::UwbMessage()
: mDirection(UwbMessageDirection::TX) {
}

UwbMessage::UwbMessage(const uint8_t* bytes, const size_t sizeBytes)
: mDirection(UwbMessageDirection::RX) {
    mBytes.resize(sizeBytes);
    mBytes.assign(bytes, bytes + sizeBytes);
}

bool UwbMessage::setSequenceNumber(const uint8_t seqNo) {
    if (mDirection == UwbMessageDirection::TX) {
        if (mBytes.size() >= MHR_SIZE) {
            mBytes[offsetof(UwbMessage::sMacHeader, sequenceNumber)] = seqNo;
            return true;
        }
    }
    return false;
}

}  // namespace uwb
}  // namespace esphome

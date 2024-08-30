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

void UwbMessage::setSequenceNumber(const uint8_t seqNo) {
    mBytes[MHR_SEQUENCE_NO_IDX] = seqNo;
}

void UwbMessage::setTargetId(const uint8_t id) {
    mBytes[COMMON_PAYLOAD_TARGET_ID_IDX] = id;
}

uint8_t UwbMessage::getTargetId() const {
    return mBytes[COMMON_PAYLOAD_TARGET_ID_IDX];
}

void UwbMessage::setSourceId(const uint8_t id) {
    mBytes[COMMON_PAYLOAD_SOURCE_ID_IDX] = id;
}

 uint8_t UwbMessage::getSourceId() const {
    return mBytes[COMMON_PAYLOAD_SOURCE_ID_IDX];
}


}  // namespace uwb
}  // namespace esphome

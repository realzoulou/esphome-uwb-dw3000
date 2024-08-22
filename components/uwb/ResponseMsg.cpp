#include "ResponseMsg.h"

 #include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace uwb {

const char* ResponseMsg::TAG = "ResponseMsg";

#define COMMON_PAYLOAD_START_BYTES {'V', 'E', 'W', 'A'}

static const ResponseMsg::sResponseFrame DEFAULT_RESPONSE_FRAME = {
    .mhr = {
        .frameControl_lsb = UwbMessage::FRAME_CONTROL_LSB,
        .frameControl_msb = UwbMessage::FRAME_CONTROL_MSB,
        .sequenceNumber = 0,
        .panId_lsb = UwbMessage::PAN_ID_LSB,
        .panId_msb = UwbMessage::PAN_ID_MSB,
    },
    .payloadCommon = {
        .commonStart = COMMON_PAYLOAD_START_BYTES,
    },
    .functionCode = ResponseMsg::RESPONSE_FCT_CODE_RANGING,
    .functionDataLen = 0x02,
    .functionData = {0,0},
    .mfr = {
        .frameCheckingSequence = 0 // CRC will be filled by the DW IC
    }
};

ResponseMsg::ResponseMsg()
: UwbMessage() {
    mBytes.resize(ResponseMsg::FRAME_SIZE);
    mBytes.assign((uint8_t*)&DEFAULT_RESPONSE_FRAME, (uint8_t*)&DEFAULT_RESPONSE_FRAME + ResponseMsg::FRAME_SIZE);
}

ResponseMsg::ResponseMsg(const uint8_t* bytes, const size_t sizeBytes)
: UwbMessage(bytes, sizeBytes) {}

bool ResponseMsg::isValid() const {
    // check size
    const auto bytes = getBytes();
    if (bytes.size() != ResponseMsg::FRAME_SIZE) {
        return false;
    }
    const auto frame = reinterpret_cast<ResponseMsg::sResponseFrame*>(getBytes().data());
    // check PAN-ID
    if (frame->mhr.panId_lsb != UwbMessage::PAN_ID_LSB) {
        return false;
    }
    if (frame->mhr.panId_msb != UwbMessage::PAN_ID_MSB) {
        return false;
    }
    // check functionCode
    if (frame->functionCode != ResponseMsg::RESPONSE_FCT_CODE_RANGING) {
        return false;
    }
    // check payload start
    const uint8_t expectedCommonPayloadStart[] = COMMON_PAYLOAD_START_BYTES;
    static_assert(UwbMessage::COMMON_PAYLOAD_START_SIZE == sizeof(expectedCommonPayloadStart), "COMMON_PAYLOAD_START_BYTES: size mismatch");
    if (std::memcmp(expectedCommonPayloadStart,
                    frame->payloadCommon.commonStart,
                    UwbMessage::COMMON_PAYLOAD_START_SIZE) != 0) {
        return false;
    }
    // check data size
    if (frame->functionDataLen != RESPONSE_DATA_SIZE) {
        return false;
    }
    return true;
}

void ResponseMsg::resetToDefault() {
    mBytes.resize(ResponseMsg::FRAME_SIZE);
    mBytes.assign((uint8_t*)&DEFAULT_RESPONSE_FRAME, (uint8_t*)&DEFAULT_RESPONSE_FRAME + ResponseMsg::FRAME_SIZE);
}

bool ResponseMsg::setFunctionCodeAndData(const uint8_t fctCode, const uint8_t* data, const size_t dataSize) {
    if (dataSize != RESPONSE_DATA_SIZE) {
        ESP_LOGE(TAG, "setFunctionCodeAndData invalid dataSize %zu", dataSize);
        return false;
    }
    const auto frame = reinterpret_cast<ResponseMsg::sResponseFrame*>(mBytes.data());
    frame->functionCode = fctCode;
    std::memcpy(frame->functionData, data, dataSize);
    return true;
}

}  // namespace uwb
}  // namespace esphome

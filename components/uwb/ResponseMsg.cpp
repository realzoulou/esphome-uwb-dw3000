#include "ResponseMsg.h"

#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace uwb {

const char* ResponseMsg::TAG = "ResponseMsg";

#define COMMON_PAYLOAD_START_BYTES_RESERVED {'R', 'P'}

static const ResponseMsg::sResponseFrame DEFAULT_RESPONSE_FRAME = {
    .mhr = {
        .frameControl_lsb = UwbMessage::FRAME_CONTROL_LSB,
        .frameControl_msb = UwbMessage::FRAME_CONTROL_MSB,
        .sequenceNumber = 0,
        .panId_lsb = UwbMessage::PAN_ID_LSB,
        .panId_msb = UwbMessage::PAN_ID_MSB,
    },
    .payloadCommon = {
        .targetId = 0x00,
        .sourceId = 0x00,
        .reserved = COMMON_PAYLOAD_START_BYTES_RESERVED,
    },
    .functionCode = ResponseMsg::RESPONSE_FCT_CODE_RANGING,
    .functionDataLen = 0x02,
    .functionData = {0,0}, // reserved for future use
    .mfr = {
        .frameCheckingSequence = 0 // CRC will be filled by the DW IC
    }
};

ResponseMsg::ResponseMsg()
: UwbMessage() {
    mBytes.resize(ResponseMsg::FRAME_SIZE);
    mBytes.assign((uint8_t*)&DEFAULT_RESPONSE_FRAME, (uint8_t*)&DEFAULT_RESPONSE_FRAME + ResponseMsg::FRAME_SIZE);
}

bool ResponseMsg::isValid() const {
    // check size
    const auto & bytes = mBytes;
    if (bytes.size() != ResponseMsg::FRAME_SIZE) {
        return false;
    }
    const auto frame = reinterpret_cast<const ResponseMsg::sResponseFrame*>(bytes.data());
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
    const uint8_t expectedCommonPayloadStart[] = COMMON_PAYLOAD_START_BYTES_RESERVED;
    static_assert(UwbMessage::COMMON_PAYLOAD_RESERVED_SIZE == sizeof(expectedCommonPayloadStart), "COMMON_PAYLOAD_START_BYTES_RESERVED: size mismatch");
    if (std::memcmp(expectedCommonPayloadStart,
                    frame->payloadCommon.reserved,
                    UwbMessage::COMMON_PAYLOAD_RESERVED_SIZE) != 0) {
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

bool ResponseMsg::fromIncomingBytes(const uint8_t* bytes, std::size_t sizeBytes) {
    if (sizeBytes == ResponseMsg::FRAME_SIZE) {
        mBytes.assign(bytes, bytes + sizeBytes);
        return true;
    } else {
        ESP_LOGW(TAG, "Incoming ResponseMsg size mismatch: got %zu, exp %zu", sizeBytes, ResponseMsg::FRAME_SIZE);
        return false;
    }
}

bool ResponseMsg::setFunctionCodeAndData(const uint8_t fctCode, const uint8_t* data, const std::size_t dataSize) {
    if (data == nullptr || dataSize != RESPONSE_DATA_SIZE) {
        ESP_LOGE(TAG, "setFunctionCodeAndData null or invalid dataSize %zu (exp %zu)", dataSize, RESPONSE_DATA_SIZE);
        return false;
    }
    const auto frame = reinterpret_cast<ResponseMsg::sResponseFrame*>(mBytes.data());
    frame->functionCode = fctCode;
    std::memcpy(frame->functionData, data, dataSize);
    return true;
}

bool ResponseMsg::getFunctionCodeAndData(uint8_t* fctCode, uint8_t* data, const std::size_t dataSize, std::size_t *actualDataSize) const {
    if (fctCode == nullptr || data == nullptr || actualDataSize == nullptr || dataSize < RESPONSE_DATA_SIZE) {
        ESP_LOGE(TAG, "getFunctionCodeAndData null's or invalid dataSize %zu (exp %zu)", dataSize, RESPONSE_DATA_SIZE);
        return false;
    }
    const auto frame = reinterpret_cast<const ResponseMsg::sResponseFrame*>(mBytes.data());
    if (frame->functionDataLen > RESPONSE_DATA_SIZE) {
        ESP_LOGE(TAG, "getFunctionCodeAndData frame's data len %zu exceeds %zu)", frame->functionDataLen, RESPONSE_DATA_SIZE);
        return false;
    }
    *fctCode = frame->functionCode;
    *actualDataSize = (std::size_t)frame->functionDataLen;
    std::memcpy(data, frame->functionData, frame->functionDataLen);
    return true;
}

}  // namespace uwb
}  // namespace esphome

#include "FinalMsg.h"

#include <cstring>

#include "esphome/core/log.h"

namespace esphome {
namespace uwb {

const char* FinalMsg::TAG = "FinalMsg";

#define COMMON_PAYLOAD_START_BYTES_RESERVED {'F', 'I'}

#define FINAL_FCT_DATA_DEFAULT { (uint8_t) 0x00, (uint8_t) 0x00 }

static const FinalMsg::sFinalFrame DEFAULT_FINAL_FRAME = {
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
    .functionCode = FinalMsg::FINAL_FCT_CODE_NO_DATA,
    .functionData = FINAL_FCT_DATA_DEFAULT,
    .initial_ts = 0,
    .response_ts = 0,
    .final_ts = 0,
    .mfr = {
        .frameCheckingSequence = 0 // CRC will be filled by the DW IC
    }
};

FinalMsg::FinalMsg()
: UwbMessage() {
    mBytes.resize(FinalMsg::FRAME_SIZE);
    mBytes.assign((uint8_t*)&DEFAULT_FINAL_FRAME, (uint8_t*)&DEFAULT_FINAL_FRAME + FinalMsg::FRAME_SIZE);
}

bool FinalMsg::isValid() const {
    // check size
    const auto & bytes = mBytes;
    if (bytes.size() != FinalMsg::FRAME_SIZE) {
        return false;
    }
    const auto frame = reinterpret_cast<const FinalMsg::sFinalFrame*>(mBytes.data());
    // check PAN-ID
    if (frame->mhr.panId_lsb != UwbMessage::PAN_ID_LSB) {
        return false;
    }
    if (frame->mhr.panId_msb != UwbMessage::PAN_ID_MSB) {
        return false;
    }
    // check functionCode
    if (   (frame->functionCode != FinalMsg::FINAL_FCT_CODE_NO_DATA)
        && (frame->functionCode != FinalMsg::FINAL_FCT_CODE_RANGING_DIST)) {
        return false;
    }
    // check payload start
    const uint8_t expectedCommonPayloadReserved[] = COMMON_PAYLOAD_START_BYTES_RESERVED;
    static_assert(UwbMessage::COMMON_PAYLOAD_RESERVED_SIZE == sizeof(expectedCommonPayloadReserved), "COMMON_PAYLOAD_RESERVED_SIZE: size mismatch");
    if (std::memcmp(expectedCommonPayloadReserved,
                    frame->payloadCommon.reserved,
                    UwbMessage::COMMON_PAYLOAD_RESERVED_SIZE) != 0) {
        return false;
    }
    // check timestamps != 0
    if ((frame->initial_ts == 0) || (frame->response_ts == 0) || (frame->final_ts == 0)) {
        return false;
    }
    return true;
}

void FinalMsg::resetToDefault() {
    mBytes.resize(FinalMsg::FRAME_SIZE);
    mBytes.assign((uint8_t*)&DEFAULT_FINAL_FRAME, (uint8_t*)&DEFAULT_FINAL_FRAME + FinalMsg::FRAME_SIZE);
}

bool FinalMsg::fromIncomingBytes(const uint8_t* bytes, std::size_t sizeBytes) {
    if (sizeBytes == FinalMsg::FRAME_SIZE) {
        mBytes.assign(bytes, bytes + sizeBytes);
        return true;
    } else {
        ESP_LOGW(TAG, "Incoming FinalMsg size mismatch: got %zu, exp %zu", sizeBytes, FinalMsg::FRAME_SIZE);
        return false;
    }
}

void FinalMsg::getTimestamps(uint32_t* initial, uint32_t* response, uint32_t* final) const {
    const auto frame = reinterpret_cast<const FinalMsg::sFinalFrame*>(mBytes.data());
    if (initial != nullptr) *initial = frame->initial_ts;
    if (response != nullptr) *response = frame->response_ts;
    if (final != nullptr) *final = frame->final_ts;
}

void FinalMsg::setTimestamps(uint32_t initial, uint32_t response, uint32_t final) {
    const auto frame = reinterpret_cast<FinalMsg::sFinalFrame*>(mBytes.data());
    frame->initial_ts = initial;
    frame->response_ts = response;
    frame->final_ts = final;
}
bool FinalMsg::setFunctionCodeAndData(const uint8_t fctCode, const uint8_t* data, const std::size_t dataSize) {
    if (data == nullptr || dataSize != FINAL_DATA_SIZE) {
        ESP_LOGE(TAG, "setFunctionCodeAndData null or invalid dataSize %zu (exp %zu)", dataSize, FINAL_DATA_SIZE);
        return false;
    }
    const auto frame = reinterpret_cast<FinalMsg::sFinalFrame*>(mBytes.data());
    frame->functionCode = fctCode;
    std::memcpy(frame->functionData, data, dataSize);
    return true;
}

bool FinalMsg::getFunctionCodeAndData(uint8_t* fctCode, uint8_t* data, const std::size_t dataSize, std::size_t *actualDataSize) const {
    if (fctCode == nullptr || data == nullptr || actualDataSize == nullptr || dataSize < FINAL_DATA_SIZE) {
        ESP_LOGE(TAG, "getFunctionCodeAndData null's or invalid dataSize %zu (exp %zu)", dataSize, FINAL_DATA_SIZE);
        return false;
    }
    const auto frame = reinterpret_cast<const FinalMsg::sFinalFrame*>(mBytes.data());
    *fctCode = frame->functionCode;
    *actualDataSize = FINAL_DATA_SIZE;
    std::memcpy(data, frame->functionData, FINAL_DATA_SIZE);
    return true;
}

}  // namespace uwb
}  // namespace esphome
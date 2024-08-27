#include "InitialMsg.h"

#include <cstring>

#include "esphome/core/log.h"
#include "esp_log.h"

namespace esphome {
namespace uwb {

const char* InitialMsg::TAG = "InitialMsg";

#define COMMON_PAYLOAD_START_BYTES {'W', 'A', 'V', 'E'}

static const InitialMsg::sInitialFrame DEFAULT_INITIAL_FRAME = {
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
    .functionCode = InitialMsg::INITIAL_FCT_CODE_RANGING,
    .mfr = {
        .frameCheckingSequence = 0 // CRC will be filled by the DW IC
    }
};

InitialMsg::InitialMsg()
: UwbMessage() {
    mBytes.resize(InitialMsg::FRAME_SIZE);
    mBytes.assign((uint8_t*)&DEFAULT_INITIAL_FRAME, (uint8_t*)&DEFAULT_INITIAL_FRAME + InitialMsg::FRAME_SIZE);
}

InitialMsg::InitialMsg(const uint8_t* bytes, const size_t sizeBytes)
: UwbMessage(bytes, sizeBytes) {}

bool InitialMsg::isValid() const {
    // check size
    const auto & bytes = mBytes;
    if (bytes.size() != InitialMsg::FRAME_SIZE) {
        ESP_LOGW(TAG, "recvd bytes %zu != %zu", bytes.size(), InitialMsg::FRAME_SIZE);
        ESP_LOG_BUFFER_HEXDUMP(TAG, bytes.data(), bytes.size(), ESP_LOG_ERROR);
        return false;
    }
    const auto frame = reinterpret_cast<const InitialMsg::sInitialFrame*>(bytes.data());
    // check PAN-ID
    if (frame->mhr.panId_lsb != UwbMessage::PAN_ID_LSB) {
        ESP_LOGW(TAG, "recvd panId_lsb 0x%02x != 0x%02x", frame->mhr.panId_lsb, UwbMessage::PAN_ID_LSB);
        return false;
    }
    if (frame->mhr.panId_msb != UwbMessage::PAN_ID_MSB) {
        ESP_LOGW(TAG, "recvd panId_msb 0x%02x != 0x%02x", frame->mhr.panId_msb, UwbMessage::PAN_ID_MSB);
        return false;
    }
    // check functionCode
    if (frame->functionCode != InitialMsg::INITIAL_FCT_CODE_RANGING) {
        ESP_LOGW(TAG, "recvd functionCode 0x%02x != 0x%02x", frame->functionCode, InitialMsg::INITIAL_FCT_CODE_RANGING);
        return false;
    }
    // check payload start
    const uint8_t expectedCommonPayloadStart[] = COMMON_PAYLOAD_START_BYTES;
    static_assert(UwbMessage::COMMON_PAYLOAD_START_SIZE == sizeof(expectedCommonPayloadStart), "COMMON_PAYLOAD_START_BYTES: size mismatch");
    if (std::memcmp(expectedCommonPayloadStart,
                    frame->payloadCommon.commonStart,
                    UwbMessage::COMMON_PAYLOAD_START_SIZE) != 0) {
        ESP_LOGW(TAG, "payloadCommon mismatch recvd vs. expected:");
        ESP_LOG_BUFFER_HEXDUMP(TAG, frame->payloadCommon.commonStart, UwbMessage::COMMON_PAYLOAD_START_SIZE, ESP_LOG_ERROR);
        ESP_LOG_BUFFER_HEXDUMP(TAG, expectedCommonPayloadStart, UwbMessage::COMMON_PAYLOAD_START_SIZE, ESP_LOG_ERROR);
        return false;
    }
    return true;
}

void InitialMsg::resetToDefault() {
    mBytes.resize(InitialMsg::FRAME_SIZE);
    mBytes.assign((uint8_t*)&DEFAULT_INITIAL_FRAME, (uint8_t*)&DEFAULT_INITIAL_FRAME + InitialMsg::FRAME_SIZE);
}

}  // namespace uwb
}  // namespace esphome
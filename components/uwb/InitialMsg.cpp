#include "InitialMsg.h"

#include <cstring>

#include "esphome/core/log.h"
#include "esp_log.h"

namespace esphome {
namespace uwb {

const char* InitialMsg::TAG = "InitialMsg";

#define COMMON_PAYLOAD_START_BYTES_RESERVED { 'I', 'N'}

static const InitialMsg::sInitialFrame DEFAULT_INITIAL_FRAME = {
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
        // likely a Final frame between other devices
        ESP_LOGV(TAG, "recvd bytes %zu != %zu", bytes.size(), InitialMsg::FRAME_SIZE);
        ESP_LOG_BUFFER_HEXDUMP(TAG, bytes.data(), bytes.size(), ESP_LOG_VERBOSE);
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
    const uint8_t expectedCommonPayloadReserved[] = COMMON_PAYLOAD_START_BYTES_RESERVED;
    static_assert(UwbMessage::COMMON_PAYLOAD_RESERVED_SIZE == sizeof(expectedCommonPayloadReserved), "COMMON_PAYLOAD_START_BYTES_RESERVED: size mismatch");
    if (std::memcmp(expectedCommonPayloadReserved,
                    frame->payloadCommon.reserved,
                    UwbMessage::COMMON_PAYLOAD_RESERVED_SIZE) != 0) {
        ESP_LOGW(TAG, "payloadCommon.reserved mismatch recvd vs. expected:");
        ESP_LOG_BUFFER_HEXDUMP(TAG, frame->payloadCommon.reserved, UwbMessage::COMMON_PAYLOAD_RESERVED_SIZE, ESP_LOG_ERROR);
        ESP_LOG_BUFFER_HEXDUMP(TAG, expectedCommonPayloadReserved, UwbMessage::COMMON_PAYLOAD_RESERVED_SIZE, ESP_LOG_ERROR);
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
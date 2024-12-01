#include "InitialMsg.h"

#include <cstring>

namespace esphome {
namespace uwb {

const char* InitialMsg::TAG = "InitialMsg";

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
        .reserved = INITIAL_PAYLOAD_START_BYTES_RESERVED,
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


bool InitialMsg::isValid() const {
    // check size
    const auto & bytes = mBytes;
    if (bytes.size() != InitialMsg::FRAME_SIZE) {
        // likely a Final frame between other devices
        std::ostringstream msg;
        msg << "recvd bytes " << +(bytes.size()) << " != " << +InitialMsg::FRAME_SIZE;
        MSG_LOGV(msg);
        ESP_LOG_BUFFER_HEXDUMP(TAG, bytes.data(), bytes.size(), ESP_LOG_VERBOSE);

        return false;
    }
    const auto frame = reinterpret_cast<const InitialMsg::sInitialFrame*>(bytes.data());
    // check PAN-ID
    if (frame->mhr.panId_lsb != UwbMessage::PAN_ID_LSB) {
        std::ostringstream msg;
        msg << "recvd panId_lsb 0x" << std::hex << +(frame->mhr.panId_lsb) << " != 0x" << +UwbMessage::PAN_ID_LSB << std::dec;
        MSG_LOGW(msg);
        return false;
    }
    if (frame->mhr.panId_msb != UwbMessage::PAN_ID_MSB) {
        std::ostringstream msg;
        msg << "recvd panId_msb 0x" << std::hex << +(frame->mhr.panId_msb) << " != 0x" << +UwbMessage::PAN_ID_MSB << std::dec;
        MSG_LOGW(msg);
        return false;
    }
    // check functionCode
    if (frame->functionCode != InitialMsg::INITIAL_FCT_CODE_RANGING) {
        std::ostringstream msg;
        msg << "recvd functionCode 0x" << std::hex << +(frame->functionCode) << " != 0x" << +InitialMsg::INITIAL_FCT_CODE_RANGING << std::dec;
        MSG_LOGW(msg);
        return false;
    }
    // check payload start
    const uint8_t expectedCommonPayloadReserved[] = INITIAL_PAYLOAD_START_BYTES_RESERVED;
    static_assert(UwbMessage::COMMON_PAYLOAD_RESERVED_SIZE == sizeof(expectedCommonPayloadReserved), "COMMON_PAYLOAD_START_BYTES_RESERVED: size mismatch");
    if (std::memcmp(expectedCommonPayloadReserved,
                    frame->payloadCommon.reserved,
                    UwbMessage::COMMON_PAYLOAD_RESERVED_SIZE) != 0) {
        std::ostringstream msg;
        msg << "payloadCommon.reserved mismatch recvd vs. expected:";
        MSG_LOGW(msg);
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

bool InitialMsg::fromIncomingBytes(const uint8_t* bytes, std::size_t sizeBytes) {
    if (sizeBytes == InitialMsg::FRAME_SIZE) {
        mBytes.assign(bytes, bytes + sizeBytes);
        return true;
    } else {
        // not a warning, this could be a Final message to a different anchor
        std::ostringstream msg;
        msg << "Incoming InitialMsg size mismatch: got " << +sizeBytes << " exp " << +InitialMsg::FRAME_SIZE;
        MSG_LOGI(msg);
        return false;
    }
}

}  // namespace uwb
}  // namespace esphome

#pragma once

#include <cstdint>
#include <vector>

#include "UwbMessage.h"

namespace esphome {
namespace uwb {

class InitialMsg : public UwbMessage {

public:
    /* Definition of the Initial frame structure */
    static const std::size_t FRAME_SIZE = MHR_SIZE + COMMON_PAYLOAD_START_SIZE + 1 + MFR_SIZE;
    typedef struct {
        UwbMessage::sMacHeader mhr;
        UwbMessage::sPayloadCommon payloadCommon;
        uint8_t functionCode;
        UwbMessage::sMacFooter mfr;
    } __attribute__((packed)) sInitialFrame;
    static_assert(sizeof(sInitialFrame) == FRAME_SIZE, "sInitialFrame: size mismatch");

    static const uint8_t INITIAL_FCT_CODE_RANGING = 0x21;

public:
    InitialMsg();
    InitialMsg(const uint8_t* bytes, size_t sizeBytes);

    virtual bool isValid() const;
    virtual void resetToDefault();

protected:
    static const char* TAG;
};

}  // namespace uwb
}  // namespace esphome
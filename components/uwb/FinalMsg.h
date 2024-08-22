#pragma once

#include <cstdint>

#include "UwbMessage.h"

namespace esphome {
namespace uwb {

class FinalMsg : public UwbMessage {

public:
    /* Definition of the Final frame structure */
    static const std::size_t FRAME_SIZE = MHR_SIZE + COMMON_PAYLOAD_START_SIZE + 1 + 3*sizeof(uint32_t) + MFR_SIZE;
    typedef struct {
        UwbMessage::sMacHeader mhr;
        UwbMessage::sPayloadCommon payloadCommon;
        uint8_t functionCode;
        uint32_t initial_ts;
        uint32_t response_ts;
        uint32_t final_ts;
        UwbMessage::sMacFooter mfr;
    } __attribute__((packed)) sFinalFrame;
    static_assert(sizeof(sFinalFrame) == FRAME_SIZE, "sFinalFrame: size mismatch");

    static const uint8_t FINAL_FCT_CODE_RANGING = 0x23;

public:
    FinalMsg();
    FinalMsg(const uint8_t* bytes, size_t sizeBytes);

    virtual bool isValid() const;
    virtual void resetToDefault();

    void getTimestamps(uint32_t* initial, uint32_t* response, uint32_t* final) const;
    void setTimestamps(uint32_t initial, uint32_t response, uint32_t final);

protected:
    static const char* TAG;
};

}  // namespace uwb
}  // namespace esphome
#pragma once

#include <cstdint>
#include <vector>

#include "UwbMessage.h"

#define INITIAL_PAYLOAD_START_BYTES_RESERVED { 'I', 'N'}

namespace esphome {
namespace uwb {

class InitialMsg : public UwbMessage {

public:
    /* Definition of the Initial frame structure */
    static const std::size_t FRAME_SIZE = MHR_SIZE + COMMON_PAYLOAD_START_SIZE + 1 + 2 + MFR_SIZE;
    typedef struct {
        UwbMessage::sMacHeader mhr;
        UwbMessage::sPayloadCommon payloadCommon;
        uint8_t functionCode;
        uint16_t functionData;
        UwbMessage::sMacFooter mfr;
    } __attribute__((packed)) sInitialFrame;
    static_assert(sizeof(sInitialFrame) == FRAME_SIZE, "sInitialFrame: size mismatch");

    static constexpr uint8_t INITIAL_FCT_CODE_RANGING               = 0x21;
    static constexpr uint8_t INITIAL_FCT_CODE_ANT_DELAY_CALIBRATION = 0x22;

public:
    InitialMsg();

    virtual bool isValid() const;
    virtual void resetToDefault();
    virtual bool fromIncomingBytes(const uint8_t* bytes, std::size_t sizeBytes);

    virtual void setFunctionCodeAndData(const uint8_t fctCode, const uint16_t data);
    virtual void getFunctionCodeAndData(uint8_t & fctCode, uint16_t & data) const;

private:
    static const char* TAG;
};

}  // namespace uwb
}  // namespace esphome

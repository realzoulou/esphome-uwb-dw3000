#pragma once

#include <cstdint>
#include <vector>
#include <stddef.h>

#include "UwbMessage.h"

namespace esphome {
namespace uwb {

class ResponseMsg : public UwbMessage {

public:
    /* Definition of the Response frame structure */
    static const std::size_t RESPONSE_DATA_SIZE = 2;
    static const std::size_t FRAME_SIZE = MHR_SIZE + COMMON_PAYLOAD_START_SIZE + 2 + RESPONSE_DATA_SIZE + MFR_SIZE;
    typedef struct {
        UwbMessage::sMacHeader mhr;
        UwbMessage::sPayloadCommon payloadCommon;
        uint8_t functionCode;
        uint8_t functionDataLen;
        uint8_t functionData[RESPONSE_DATA_SIZE];
        UwbMessage::sMacFooter mfr;
    } __attribute__((packed)) sResponseFrame;
    static_assert(sizeof(sResponseFrame) == FRAME_SIZE, "sResponseFrame: size mismatch");

    static const uint8_t RESPONSE_FCT_CODE_RANGING = 0x10;

public:
    ResponseMsg();

    virtual bool isValid() const;
    virtual void resetToDefault();
    virtual bool fromIncomingBytes(const uint8_t* bytes, std::size_t sizeBytes);

    virtual bool setFunctionCodeAndData(const uint8_t fctCode, const uint8_t* data, const std::size_t dataSize);
    virtual bool getFunctionCodeAndData(uint8_t* fctCode, uint8_t*data, const std::size_t dataSize, std::size_t *actualDataSize) const;

private:
    static const char* TAG;
};

}  // namespace uwb
}  // namespace esphome

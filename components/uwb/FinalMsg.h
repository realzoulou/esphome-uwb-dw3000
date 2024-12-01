#pragma once

#include <cstdint>

#include "UwbMessage.h"

#define FINAL_PAYLOAD_START_BYTES_RESERVED {'F', 'I'}
#define FINAL_FCT_DATA_DEFAULT             { (uint8_t) 0x00, (uint8_t) 0x00 }

namespace esphome {
namespace uwb {

class FinalMsg : public UwbMessage {

public:
    static const uint8_t FINAL_FCT_CODE_NO_DATA;
    static const uint8_t FINAL_FCT_CODE_RANGING_DIST;
    static constexpr std::size_t FINAL_DATA_SIZE = 2;

    /* Definition of the Final frame structure */
    static const std::size_t FRAME_SIZE = MHR_SIZE + COMMON_PAYLOAD_START_SIZE + 1 + FINAL_DATA_SIZE + 3*sizeof(uint32_t) + MFR_SIZE;
    typedef struct {
        UwbMessage::sMacHeader mhr;
        UwbMessage::sPayloadCommon payloadCommon;
        uint8_t functionCode;
        uint8_t functionData[FINAL_DATA_SIZE];
        uint32_t initial_ts;
        uint32_t response_ts;
        uint32_t final_ts;
        UwbMessage::sMacFooter mfr;
    } __attribute__((packed)) sFinalFrame;
    static_assert(sizeof(sFinalFrame) == FRAME_SIZE, "sFinalFrame: size mismatch");

public:
    FinalMsg();

    virtual bool isValid() const;
    virtual void resetToDefault();
    virtual bool fromIncomingBytes(const uint8_t* bytes, std::size_t sizeBytes);

    void getTimestamps(uint32_t* initial, uint32_t* response, uint32_t* final) const;
    void setTimestamps(uint32_t initial, uint32_t response, uint32_t final);

    virtual bool setFunctionCodeAndData(const uint8_t fctCode, const uint8_t* data, const std::size_t dataSize);
    virtual bool getFunctionCodeAndData(uint8_t* fctCode, uint8_t*data, const std::size_t dataSize, std::size_t *actualDataSize) const;

private:
    static const char* TAG;
};

}  // namespace uwb
}  // namespace esphome

/*
 * Copyright 2024 realzoulou
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cstdint>
#include <vector>
#include <stddef.h>

#include "UwbMessage.h"

#define RESPONSE_PAYLOAD_START_BYTES_RESERVED {'R', 'P'}

namespace esphome {
namespace uwb {

class ResponseMsg : public UwbMessage {

public:
    /* Definition of the Response frame structure */
    static constexpr std::size_t RESPONSE_DATA_SIZE = 2;
    static const std::size_t FRAME_SIZE = MHR_SIZE + COMMON_PAYLOAD_START_SIZE + 2 + RESPONSE_DATA_SIZE + MFR_SIZE;
    static const uint8_t RESPONSE_FCT_CODE_RANGING;
    static const uint8_t RESPONSE_FCT_CODE_ANT_DELAY_CALIBRATION;
    typedef struct {
        UwbMessage::sMacHeader mhr;
        UwbMessage::sPayloadCommon payloadCommon;
        uint8_t functionCode;
        uint8_t functionDataLen;
        uint8_t functionData[RESPONSE_DATA_SIZE];
        UwbMessage::sMacFooter mfr;
    } __attribute__((packed)) sResponseFrame;
    static_assert(sizeof(sResponseFrame) == FRAME_SIZE, "sResponseFrame: size mismatch");

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

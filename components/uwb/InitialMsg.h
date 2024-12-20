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

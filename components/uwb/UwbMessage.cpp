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

#include "UwbMessage.h"

#include <cstddef>

namespace esphome {
namespace uwb {

const char* UwbMessage::TAG = "UwbMessage";

UwbMessage::UwbMessage() {
}

void UwbMessage::setSequenceNumber(const uint8_t seqNo) {
    mBytes[MHR_SEQUENCE_NO_IDX] = seqNo;
}

void UwbMessage::setTargetId(const uint8_t id) {
    mBytes[COMMON_PAYLOAD_TARGET_ID_IDX] = id;
}

uint8_t UwbMessage::getTargetId() const {
    return mBytes[COMMON_PAYLOAD_TARGET_ID_IDX];
}

void UwbMessage::setSourceId(const uint8_t id) {
    mBytes[COMMON_PAYLOAD_SOURCE_ID_IDX] = id;
}

 uint8_t UwbMessage::getSourceId() const {
    return mBytes[COMMON_PAYLOAD_SOURCE_ID_IDX];
}


}  // namespace uwb
}  // namespace esphome

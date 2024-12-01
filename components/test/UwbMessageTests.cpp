#include <gtest/gtest.h>

#include <cstring>

#include "InitialMsg.h"
#include "ResponseMsg.h"
#include "FinalMsg.h"

using namespace esphome::uwb;

TEST(UwbMessage, constructor) {
    InitialMsg imsg;
    ResponseMsg rmsg;
    FinalMsg fmsg;
}

TEST(UwbMessage, isValid) {
    InitialMsg imsg;
    ResponseMsg rmsg;
    FinalMsg fmsg;

    EXPECT_TRUE(imsg.isValid());
    EXPECT_TRUE(rmsg.isValid());
    EXPECT_FALSE(fmsg.isValid()); // due to timestamps
}

TEST(UwbMessage, resetToDefaults) {
    InitialMsg imsg;
    ResponseMsg rmsg;
    FinalMsg fmsg;

    imsg.resetToDefault();
    rmsg.resetToDefault();
    fmsg.resetToDefault();

    EXPECT_TRUE(imsg.isValid());
    EXPECT_TRUE(rmsg.isValid());
    EXPECT_FALSE(fmsg.isValid()); // due to timestamps
}

TEST(UwbMessage, sourceAndtargetId) {
    InitialMsg imsg;
    ResponseMsg rmsg;
    FinalMsg fmsg;

    imsg.setSourceId(10);
    EXPECT_EQ(imsg.getSourceId(), 10);
    imsg.setTargetId(11);
    EXPECT_EQ(imsg.getTargetId(), 11);
    EXPECT_TRUE(imsg.isValid());

    rmsg.setSourceId(20);
    EXPECT_EQ(rmsg.getSourceId(), 20);
    rmsg.setTargetId(21);
    EXPECT_EQ(rmsg.getTargetId(), 21);
    EXPECT_TRUE(rmsg.isValid());

    fmsg.setSourceId(30);
    EXPECT_EQ(fmsg.getSourceId(), 30);
    fmsg.setTargetId(31);
    EXPECT_EQ(fmsg.getTargetId(), 31);
    EXPECT_FALSE(fmsg.isValid()); // due to timestamps
}

TEST(UwbMessage, sequenceNumber) {
    InitialMsg imsg;
    ResponseMsg rmsg;
    FinalMsg fmsg;

    imsg.setSequenceNumber(40);
    EXPECT_TRUE(imsg.isValid());
    rmsg.setSequenceNumber(41);
    EXPECT_TRUE(rmsg.isValid());
    fmsg.setSequenceNumber(42);
    EXPECT_FALSE(fmsg.isValid()); // due to timestamps
}

TEST(UwbMessage, InitialMsg_fromIncomingBytes) {
    const InitialMsg::sInitialFrame INCOMING_INIITAL = {
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
            .frameCheckingSequence = 0
        }
    };

    InitialMsg imsg;

    const auto payload = reinterpret_cast<const uint8_t*>(&INCOMING_INIITAL);
    EXPECT_TRUE(imsg.fromIncomingBytes(payload, sizeof(INCOMING_INIITAL)));

    EXPECT_TRUE(imsg.isValid());
    EXPECT_TRUE(std::memcmp(payload, imsg.getBytes().data(), sizeof(INCOMING_INIITAL)) == 0);
}
TEST(UwbMessage, ResponseMsg_fromIncomingBytes) {
    const ResponseMsg::sResponseFrame INCOMING_RESPONSE = {
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
            .reserved = RESPONSE_PAYLOAD_START_BYTES_RESERVED,
        },
        .functionCode = ResponseMsg::RESPONSE_FCT_CODE_RANGING,
        .functionDataLen = 0x02,
        .functionData = {0,0},
        .mfr = {
            .frameCheckingSequence = 0
        }
    };

    ResponseMsg rmsg;

    const auto payload = reinterpret_cast<const uint8_t*>(&INCOMING_RESPONSE);
    EXPECT_TRUE(rmsg.fromIncomingBytes(payload, sizeof(INCOMING_RESPONSE)));

    EXPECT_TRUE(rmsg.isValid());
    EXPECT_TRUE(std::memcmp(payload, rmsg.getBytes().data(), sizeof(INCOMING_RESPONSE)) == 0);
}
TEST(UwbMessage, FinalMsg_fromIncomingBytes) {
    const FinalMsg::sFinalFrame INCOMING_FINAL = {
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
            .reserved = FINAL_PAYLOAD_START_BYTES_RESERVED,
        },
        .functionCode = FinalMsg::FINAL_FCT_CODE_NO_DATA,
        .functionData = FINAL_FCT_DATA_DEFAULT,
        .initial_ts = 1,
        .response_ts = 1,
        .final_ts = 1,
        .mfr = {
            .frameCheckingSequence = 0
        }
    };

    FinalMsg fmsg;

    const auto payload = reinterpret_cast<const uint8_t*>(&INCOMING_FINAL);
    EXPECT_TRUE(fmsg.fromIncomingBytes(payload, sizeof(INCOMING_FINAL)));

    EXPECT_TRUE(fmsg.isValid());
    EXPECT_TRUE(std::memcmp(payload, fmsg.getBytes().data(), sizeof(INCOMING_FINAL)) == 0);
}

TEST(UwbMessage, ResponseMsg_functionCodeAndData) {
    ResponseMsg rmsg;

    const uint8_t data2[] = {47, 11};
    uint8_t databuf[2];
    uint8_t fctCode;
    std::size_t actualSize;

    EXPECT_TRUE(rmsg.setFunctionCodeAndData(ResponseMsg::RESPONSE_FCT_CODE_RANGING, data2, sizeof(data2)));
    EXPECT_TRUE(rmsg.getFunctionCodeAndData(&fctCode, databuf, sizeof(databuf), &actualSize));
    EXPECT_EQ(fctCode, ResponseMsg::RESPONSE_FCT_CODE_RANGING);
    EXPECT_EQ(actualSize, sizeof(databuf));
    EXPECT_EQ(databuf[0], data2[0]);
    EXPECT_EQ(databuf[1], data2[1]);

    const uint8_t data3[] = {0,0,0};
    EXPECT_FALSE(rmsg.setFunctionCodeAndData(ResponseMsg::RESPONSE_FCT_CODE_RANGING, data3, sizeof(data3)));

    const uint8_t data1[] = {0};
    EXPECT_FALSE(rmsg.setFunctionCodeAndData(ResponseMsg::RESPONSE_FCT_CODE_RANGING, data1, sizeof(data1)));

    EXPECT_FALSE(rmsg.setFunctionCodeAndData(ResponseMsg::RESPONSE_FCT_CODE_RANGING, nullptr, 0));
}
TEST(UwbMessage, FinalMsg_functionCodeAndData) {
    FinalMsg fmsg;

    const uint8_t data21[] = {8, 15};
    const uint8_t data22[] = {47, 11};
    uint8_t databuf[2];
    uint8_t fctCode;
    std::size_t actualSize;

    EXPECT_TRUE(fmsg.setFunctionCodeAndData(FinalMsg::FINAL_FCT_CODE_NO_DATA, data21, sizeof(data21)));
    EXPECT_TRUE(fmsg.getFunctionCodeAndData(&fctCode, databuf, sizeof(databuf), &actualSize));
    EXPECT_EQ(fctCode, FinalMsg::FINAL_FCT_CODE_NO_DATA);
    EXPECT_EQ(actualSize, sizeof(databuf));
    EXPECT_EQ(databuf[0], data21[0]);
    EXPECT_EQ(databuf[1], data21[1]);

    EXPECT_TRUE(fmsg.setFunctionCodeAndData(FinalMsg::FINAL_FCT_CODE_RANGING_DIST, data22, sizeof(data22)));
    EXPECT_TRUE(fmsg.getFunctionCodeAndData(&fctCode, databuf, sizeof(databuf), &actualSize));
    EXPECT_EQ(fctCode, FinalMsg::FINAL_FCT_CODE_RANGING_DIST);
    EXPECT_EQ(actualSize, sizeof(databuf));
    EXPECT_EQ(databuf[0], data22[0]);
    EXPECT_EQ(databuf[1], data22[1]);

    const uint8_t data3[] = {0,0,0};
    EXPECT_FALSE(fmsg.setFunctionCodeAndData(FinalMsg::FINAL_FCT_CODE_NO_DATA, data3, sizeof(data3)));

    const uint8_t data1[] = {0};
    EXPECT_FALSE(fmsg.setFunctionCodeAndData(FinalMsg::FINAL_FCT_CODE_RANGING_DIST, data1, sizeof(data1)));

    EXPECT_FALSE(fmsg.setFunctionCodeAndData(FinalMsg::FINAL_FCT_CODE_NO_DATA, nullptr, 0));
    EXPECT_FALSE(fmsg.setFunctionCodeAndData(FinalMsg::FINAL_FCT_CODE_RANGING_DIST, nullptr, 0));
}

TEST(UwbMessage, FinalMsg_timestamps) {
    FinalMsg fmsg;

    const uint32_t i = 815, r = 4711, f = 42;
    uint32_t i_, r_, f_;

    fmsg.setTimestamps(i, r, f);
    fmsg.getTimestamps(nullptr, nullptr, nullptr);
    fmsg.getTimestamps(&i_, &r_, &f_);
    EXPECT_EQ(i, i_);
    EXPECT_EQ(r, r_);
    EXPECT_EQ(f, f_);
    EXPECT_TRUE(fmsg.isValid());
}

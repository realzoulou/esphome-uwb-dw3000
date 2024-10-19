#pragma once

#include <cstdint>
#include <memory>
#include <vector>
#include <stddef.h>

namespace esphome {
namespace uwb {

class UwbMessage {
public:
    /* Structure of a UWB frame (start) */
    /* --- MAC Header MHR --- */
    static const std::size_t MHR_SIZE = 5;
    typedef struct {
        // Frame Control = 2 octets = 16 bits, but transferred Least Significant Byte First
        uint8_t frameControl_lsb;
        uint8_t frameControl_msb;
        // Sequence number (modulo 256)
        uint8_t sequenceNumber;
        // Destination PAN identifier "DECA" = 2 octets = 16 bit, but transferred Least Significant Byte First
        uint8_t panId_lsb;
        uint8_t panId_msb;
    } __attribute__((packed)) sMacHeader;
    static const std::size_t MHR_SEQUENCE_NO_IDX = offsetof(UwbMessage::sMacHeader, sequenceNumber);
    static_assert(sizeof(sMacHeader) == MHR_SIZE, "sMacHeader: size mismatch");
    // Common payload start bytes
    static const std::size_t COMMON_PAYLOAD_START_SIZE = 4;
    static const std::size_t COMMON_PAYLOAD_RESERVED_SIZE = 2;
    typedef struct {
        uint8_t targetId; // ID of targeted device
        uint8_t sourceId; // ID of source device
        uint8_t reserved[COMMON_PAYLOAD_RESERVED_SIZE];
    } __attribute__((packed)) sPayloadCommon;
    static_assert(sizeof(sPayloadCommon) == COMMON_PAYLOAD_START_SIZE, "sPayloadCommon: size mismatch");
    static const std::size_t COMMON_PAYLOAD_TARGET_ID_IDX = sizeof(sMacHeader) + offsetof(UwbMessage::sPayloadCommon, targetId);
    static const std::size_t COMMON_PAYLOAD_SOURCE_ID_IDX = sizeof(sMacHeader) + offsetof(UwbMessage::sPayloadCommon, sourceId);
    static const std::size_t MFR_SIZE = 2;
    typedef struct {
        // Frame Checking Sequence (FCS) = CRC filled by the DW IC, no need to fill it in SW, but reserve the 2 bytes!
        uint16_t frameCheckingSequence;
    } __attribute__((packed)) sMacFooter;
    static_assert(sizeof(sMacFooter) == MFR_SIZE, "sMacFooter: size mismatch");
    /* Structure of a UWB frame (end) */

    static const uint8_t FRAME_CONTROL_LSB
        = 0x41; // 0b 0 1 0 0 0 001 : Frame Control octet 2
                //              ^^^ bit 2-0 = Frame Type : 0b001 = Data
                //            ^ bit 3 = Security Enabled = not enabled
                //          ^ bit 4 = Frame Pending : 0b0 = indicates that the sending device has no more data for the recipient
                //        ^ bit 5 = ACK Request : 0b0 = recipient device should not send an IEEE802.15.4 immediate acknowledgment frame (Imm-Ack)
                //      ^ bit 6 = PAN ID Compress : 0b1 = If the PAN ID compression bit is set to one and both the source and destination addresses are present,
                //                                        the frame shall contain only the Destination PAN Identifier field,
                //                                        and the Source PAN Identifier field shall be assumed equal to that of the destination.
                //    ^ bit 7 = Reserved
    static const uint8_t FRAME_CONTROL_MSB
        =  0x88; // 0b 10 00 10 00 : MAC Header MHR, Frame Control octet 1
                 //             ^^ bits 9-8 = Reserved
                 //          ^^ bits 11&10 = Destination Address Mode : 0b10 = The destination address field is a short (16-bit) address.
                 //       ^^ bits 13&12 = Frame Version : 0b00 to indicate a frame compatible with IEEE 802.15.4-2003 and 0b01 to indicate an IEEE 802.15.4 frame
                 //    ^^ bits 15&14 = Source Address mode : 0b10 = The source address field is a short (16-bit) address.

    static const uint8_t PAN_ID_LSB = 0xCA;
    static const uint8_t PAN_ID_MSB = 0xDE;

public:
    UwbMessage();

    virtual bool isValid() const = 0;
    virtual void resetToDefault() = 0;
    virtual bool fromIncomingBytes(const uint8_t* bytes, std::size_t sizeBytes) = 0;

    inline std::vector<uint8_t> & getBytes() { return mBytes; }
    virtual void setSequenceNumber(const uint8_t seqNo);
    virtual void setTargetId(const uint8_t id);
    virtual uint8_t getTargetId() const;
    virtual void setSourceId(const uint8_t id);
    virtual uint8_t getSourceId() const;

protected:
    std::vector<uint8_t> mBytes;

private:
    static const char* TAG;
};

}  // namespace uwb
}  // namespace esphome

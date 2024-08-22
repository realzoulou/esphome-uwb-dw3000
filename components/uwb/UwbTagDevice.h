#pragma once

#include "Dw3000Device.h"
#include "InitialMsg.h"
#include "FinalMsg.h"
#include "ResponseMsg.h"

#include "esphome/core/helpers.h"

namespace esphome {
namespace uwb {

class UwbTagDevice : public Dw3000Device {

    typedef enum {
        MYSTATE_UNKNOWN,
        // Initial frame to anchor(s)
        MYSTATE_PREPARE_SEND_INITIAL,
        MYSTATE_SENT_INITIAL,
        MYSTATE_SEND_ERROR_INITIAL,
        // Response frame from an anchor
        MYSTATE_WAIT_RECV_RESPONSE,
        MYSTATE_RECVD_FRAME,
        MYSTATE_RECVD_VALID_RESPONSE,
        MYSTATE_RECVD_INVALID_RESPONSE,
        // Final frame to responder
        MYSTATE_SENT_FINAL,
        MYSTATE_SEND_ERROR_FINAL,
    } eMyState;

    /* Time in millis between two ranging Initial messages. */
    static const uint32_t RANGING_INTERVAL_MS = 1000;

    /* Delay between frames, in UWB microseconds.*/

    /* This is the delay used in dwt_setrxaftertxdelay() from the end of the frame transmission to the enable of the receiver,
       as programmed for the DW IC's wait for response feature. */
    static const uint32_t INITIAL_TX_TO_RESP_RX_DLY_UUS = 700;

    /* This is the delay used in dwt_setdelayedtrxtime() from frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function.
       Adjusting this value lower and lower until dwt_starttx() starts returning DWT_ERROR status allows the user to tweak their system to calculate the
       shortest turn-around time for messages. */
    static const uint32_t RESP_RX_TO_FINAL_TX_DLY_UUS   = 720;

    /* Receive response timeout. This is the delay used in dwt_setrxtimeout().
       The time parameter used here is in 1.0256 us (UWB microseconds, i.e. 512/499.2 MHz) units.
       The maximum RX timeout is ~ 1.0754s. */
    static const uint32_t RESP_RX_TIMEOUT_UUS           = 300;

    /* Preamble timeout, in multiple of PAC size. */
    static const uint32_t PRE_TIMEOUT                   = 5;

    /* Maximum duration in millis for blocking loop() doing polling for incoming frames. */
    static const uint32_t MAX_POLL_DURATION_MS = 25;

    /*  How long to wait in MYSTATE_WAIT_RECV_RESPONSE */
    static const uint32_t WAIT_RESPONSE_RX_TIMEOUT_MS = 100;
    static_assert(WAIT_RESPONSE_RX_TIMEOUT_MS < RANGING_INTERVAL_MS, "WAIT_RESPONSE_RX_TIMEOUT_MS must be < RANGING_INTERVAL_MS");

public:
    UwbTagDevice();
    ~UwbTagDevice();

    virtual void setup();
    virtual void loop();

protected:
    virtual void setMyState(const eMyState state);

    virtual void do_ranging();

    virtual void prepareSendInitial();
    virtual void sentInitial();
    virtual void sendInitialError();
    virtual void waitRecvResponse();
    virtual void recvdFrameResponse();
    virtual void recvdValidResponse();
    virtual void recvdInvalidResponse();
    virtual void sentFinal();
    virtual void sendErrorFinal();

protected:
    static const char* TAG;
    static const char* STATE_TAG;

    eMyState prevState{MYSTATE_UNKNOWN};
    eMyState currState{MYSTATE_UNKNOWN};

    /* Buffer to store received frames. */
    static const std::size_t RX_BUF_LEN{ResponseMsg::FRAME_SIZE};
    uint8_t* rx_buffer{nullptr};

    /* Current Initial Frame. */
    InitialMsg mInitialFrame;

    /* Current Final frame. */
    FinalMsg mFinalFrame;

    /* millis() of when last sent Initial frame out successfully. */
    uint32_t mLastInitialSentMillis{0};

    /* micros() of when entered waiting for Response. */
    uint32_t mEnteredWaitRecvResponseMicros{0};

    /* DW IC SYS_TIME timestamp of when Initial frame was sent. */
    uint64_t mInitial_tx_ts{0};

    HighFrequencyLoopRequester mHighFreqLoopRequester;
};

}  // namespace uwb
}  // namespace esphome

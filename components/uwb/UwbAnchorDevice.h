#pragma once

#include <vector>

#include "Dw3000Device.h"
#include "ResponseMsg.h"
#include "FinalMsg.h"

#include "esphome/core/helpers.h"

namespace esphome {
namespace uwb {

class UwbAnchorDevice : public Dw3000Device {

public:
    typedef enum {
        MYSTATE_UNKNOWN,
        // Initial message from tag
        MYSTATE_PREPARE_WAIT_RECV_INITIAL,
        MYSTATE_WAIT_RECV_INITIAL,
        MYSTATE_RECVD_FRAME_INITIAL,
        MYSTATE_RECVD_FRAME_VALID_INITIAL,
        MYSTATE_RECVD_FRAME_INVALID_INITIAL,
        // Response message to tag
        MYSTATE_SENT_RESPONSE,
        MYSTATE_SEND_ERROR_RESPONSE,
        // Final message from tag for range calculation
        MYSTATE_WAIT_RECV_FINAL,
        MYSTATE_RECVD_FRAME_FINAL,
        MYSTATE_RECVD_FRAME_VALID_FINAL,
        MYSTATE_RECVD_FRAME_INVALID_FINAL,
        // Final message to tag for its range calculation
        MYSTATE_SENT_FINAL,
        MYSTATE_SEND_ERROR_FINAL,
    } eMyState;

    /* Delays between frames, in UWB microseconds. */

    /* This is the delay used with dwt_setrxaftertxdelay() from the end of the frame transmission to the enable of the receiver,
       as programmed for the DW IC's wait for response feature. */
    static const uint32_t RESP_TX_TO_FINAL_RX_DLY_UUS   = 700;

    /* This is the delay used with dwt_setdelayedtrxtime() from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function.
       This includes the frame length of approximately 190 us with current configuration.
       Adjusting this value lower and lower until dwt_starttx() starts returning DWT_ERROR status allows the user to tweak their system to calculate the
       shortest turn-around time for messages.
       If increasing this value, also the Tag timeouts may need to be adjusted. */
    static const uint32_t INITIAL_RX_TO_RESP_TX_DLY_UUS =  890;
    static const uint32_t FINAL_RX_TO_FINAL_TX_DLY_UUS  = 1650;

    /* Receive Final message timeout. This is the delay used in dwt_setrxtimeout().
       The time parameter used here is in 1.0256 us (UWB microseconds, i.e. 512/499.2 MHz) units.
       The maximum RX timeout is ~ 1.0754s. */
    static const uint32_t FINAL_RX_TIMEOUT_UUS          = 1000;

    /* Preamble timeout, in multiple of PAC size. */
    static const uint32_t PRE_TIMEOUT                   = 0; // disable Preamble timeout

    /* Maximum duration in millis for blocking loop() doing polling for incoming frames. */
    static const uint32_t MAX_POLL_DURATION_MS = 25;

    /* How long to wait in MYSTATE_WAIT_RECV_FINAL */
    static const uint32_t WAIT_FINAL_RX_TIMEOUT_MS = 100;

public:
    UwbAnchorDevice();
    ~UwbAnchorDevice();

    virtual void setup();
    virtual void loop();

    /* get last distance in [m] and time in [ms] when last updated */
    virtual double getLastDistance(uint32_t* timeMillis) const;
    /* set last distance in [m] */
    virtual void setLastDistance(const double distance);

protected:
    virtual void setMyState(const eMyState state);

    virtual void do_ranging();

    virtual void prepareWaitRecvInitial();
    virtual void waitRecvInitial();
    virtual void recvdFrameInitial();
    virtual void recvdFrameValidInitial();
    virtual void recvdFrameInvalidInitial();
    virtual void sentResponse();
    virtual void sendErrorResponse();
    virtual void waitRecvFinal();
    virtual void recvdFrameFinal();
    virtual void recvdFrameValidFinal();
    virtual void recvdFrameInvalidFinal();
    virtual void sentFinal();
    virtual void sendErrorFinal();

protected:
    static const char* TAG;
    static const char* STATE_TAG;

    uint8_t mCurrentTagId{0xFF};

    /* Last measured distance. */
    double mLastDistance{0.0};
    /* millis() of when last distance was updated. */
    uint32_t mLastDistanceUpdatedMs{0};

    eMyState prevState{MYSTATE_UNKNOWN};
    eMyState currState{MYSTATE_UNKNOWN};

    /* Buffer to store received frames. */
    const std::size_t RX_BUF_LEN;
    uint8_t* rx_buffer{nullptr};

    /* DW IC timestamp when Intial frame received. */
    uint64_t mInitial_rx_ts{0};

    /* Current Response frame. */
    ResponseMsg mResponseFrame;

    /* Current Final response frame. */
    FinalMsg mFinalResponseFrame;

    /* micros() of when entered MYSTATE_WAIT_RECV_FINAL */
    uint64_t mEnteredWaitRecvFinalMicros{0};

    HighFrequencyLoopRequester mHighFreqLoopRequester;
};

}  // namespace uwb
}  // namespace esphome

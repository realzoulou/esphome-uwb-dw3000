#pragma once

#include "Dw3000Device.h"
#include "UwbAnchorData.h"
#include "InitialMsg.h"
#include "FinalMsg.h"
#include "ResponseMsg.h"

namespace esphome {
namespace uwb {

class UwbTagDevice : public Dw3000Device {

    typedef enum _eMyState {
        MYSTATE_UNKNOWN,
        // wait state for next ranging interval
        MYSTATE_WAIT_NEXT_RANGING_INTERVAL,
        // wait state until next anchor within a ranging interval
        MYSTATE_WAIT_NEXT_ANCHOR_RANGING,
        // Initial frame to anchor
        MYSTATE_PREPARE_SEND_INITIAL,
        MYSTATE_SENT_INITIAL,
        MYSTATE_SEND_ERROR_INITIAL,
        // Response frame from anchor
        MYSTATE_WAIT_RECV_RESPONSE,
        MYSTATE_RECVD_FRAME_RESPONSE,
        MYSTATE_RECVD_VALID_RESPONSE,
        MYSTATE_RECVD_INVALID_RESPONSE,
        // Final frame to anchor
        MYSTATE_SENT_FINAL,
        MYSTATE_SEND_ERROR_FINAL,
        // Final response frame from anchor
        MYSTATE_WAIT_RECV_FINAL,
        MYSTATE_RECVD_FRAME_FINAL,
        MYSTATE_RECVD_VALID_FINAL,
        MYSTATE_RECVD_INVALID_FINAL,
        // Ranging is done (may or may not be successful)
        MYSTATE_RANGING_DONE,
        // if ranging was successful, calculate location
        MYSTATE_CALCULATE_LOCATION,
    } eMyState;
    static const eMyState MY_DEFAULT_STATE = MYSTATE_WAIT_NEXT_RANGING_INTERVAL;

    /* Delay between frames, in UWB microseconds.*/

    /* This is the delay used with dwt_setrxaftertxdelay() from the end of the frame transmission to the enable of the receiver,
       as programmed for the DW IC's wait for response feature. */
    static const uint32_t INITIAL_TX_TO_RESP_RX_DLY_UUS         = 700;
    static const uint32_t FINAL_TX_TO_FINAL_RESPONSE_RX_DLY_UUS = 700;

    /* This is the delay used with dwt_setdelayedtrxtime() from frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function.
       Adjusting this value lower and lower until dwt_starttx() starts returning DWT_ERROR status allows the user to tweak their system to calculate the
       shortest turn-around time for messages.
       If increasing this value, also the Anchor timeouts for receiving Final frame must be adjusted. */
    static const uint32_t RESP_RX_TO_FINAL_TX_DLY_UUS   = 830;

    /* Receive response timeout. This is the delay used in dwt_setrxtimeout().
       The time parameter used here is in 1.0256 us (UWB microseconds, i.e. 512/499.2 MHz) units.
       The maximum RX timeout is ~ 1.0754s. */
    static const uint32_t RESP_RX_TIMEOUT_UUS           = 1000;
    static const uint32_t FINAL_RESPONSE_RX_TIMEOUT_UUS = 1000;

    /* Preamble timeout, in multiple of PAC size. */
    static const uint32_t PRE_TIMEOUT                   = 0; // disable Preamble timeout

    /* Maximum duration in millis for blocking loop() doing polling for incoming frames. */
    static const uint32_t MAX_POLL_DURATION_MS = 25;

    /*  How long to wait in MYSTATE_WAIT_RECV_RESPONSE/FINAL */
    static const uint32_t WAIT_RX_TIMEOUT_MS = 100;

public:
    UwbTagDevice(const std::vector<std::shared_ptr<UwbAnchorData>> & anchors,
                 const uint32_t rangingIntervalMs, const uint32_t maxAgeAnchorDistanceMs,
                 sensor::Sensor* latitudeSensor, sensor::Sensor* longitudeSensor,
                 sensor::Sensor* locationErrorEstimateSensor, sensor::Sensor* anchorsInUseSensor);

    ~UwbTagDevice();

    virtual void setup();
    virtual void loop();

protected:
    virtual void setMyState(const eMyState state);

    virtual void do_ranging();

    virtual void waitNextRangingInterval();
    virtual void waitNextAnchorRanging();
    virtual void prepareSendInitial();
    virtual void sentInitial();
    virtual void sendInitialError();
    virtual void waitRecvResponse();
    virtual void recvdFrameResponse();
    virtual void recvdValidResponse();
    virtual void recvdInvalidResponse();
    virtual void sentFinal();
    virtual void sendErrorFinal();
    virtual void waitRecvFinal();
    virtual void recvdFrameFinal();
    virtual void recvdValidFinal();
    virtual void recvdInvalidFinal();
    virtual void rangingDone(bool success, double distance=NAN);
    virtual void calculateLocation();
protected:
    static const char* TAG;
    static const char* STATE_TAG;

    /* Time in millis between two ranging Initial messages. */
    const uint32_t RANGING_INTERVAL_MS;
    /* Maximum age of distance to anchor until anchor is considered 'away'. */
    const uint32_t MAX_AGE_ANCHOR_DISTANCE_MS;

    /* Array of all Anchors that this tag shall do ranging with. */
    std::vector<std::shared_ptr<UwbAnchorData>> mAnchors;
    /* Array of all Anchors: count of remaining attempts. */
    std::vector<uint8_t> mAnchorCurrentRangingSuccess;
    /* Index into mAnchors of current Anchor to do ranging with. -1 if there is no Anchor configured. */
    int mCurrentAnchorIndex{-1};

    /* Sensors. */
    sensor::Sensor *mLatitudeSensor,
                   *mLongitudeSensor,
                   *mLocationErrorEstimateSensor,
                   *mAnchorsInUseSensor;

    eMyState prevState{MYSTATE_UNKNOWN};
    eMyState currState{MYSTATE_UNKNOWN};

    /* Buffer to store received frames. */
    const std::size_t RX_BUF_LEN;
    uint8_t* rx_buffer{nullptr};

    /* Current Initial Frame. */
    InitialMsg mInitialFrame;

    /* Current Response Frame. */
    ResponseMsg mResponseFrame;

    /* Current Final Frame. */
    FinalMsg mFinalFrame;

    /* DW IC SYS_TIME timestamp of when Initial frame was sent. */
    uint64_t mInitial_tx_ts{0};

    /* DW IC timestamp when Response frame received. */
    uint64_t mResponse_rx_ts{0};

    /* millis() of when last ranging interval started. */
    uint32_t mLastRangingIntervalStartedMillis{0};

    /* millis() of when last sent Initial frame out successfully. */
    uint32_t mLastInitialSentMillis{0};

    /* micros() of when entered waiting for Response. */
    uint32_t mEnteredWaitRecvResponseMicros{0};

    /* micros() of when entered waiting for Final response. */
    uint32_t mEnteredWaitRecvFinalMicros{0};

};

}  // namespace uwb
}  // namespace esphome

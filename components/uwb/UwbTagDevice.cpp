#include "UwbTagDevice.h"

#include <iomanip>
#include <sstream>

#include "InitialMsg.h"
#include "ResponseMsg.h"
#include "FinalMsg.h"
#include "Location.h"

#include "esphome/core/log.h"

#include "dw3000.h"

#define LOG_ANCHORDATA_TO_STREAM(ostream, /*UwbAnchorData*/ anchor) \
    ostream << std::hex << +anchor->getId() << std::dec << "("; \
    ostream << std::setprecision(10); \
    ostream << +anchor->getLatitude() << "," << +anchor->getLongitude() << ")"

namespace esphome {
namespace uwb {

const char* UwbTagDevice::TAG = "tag";
const char* UwbTagDevice::STATE_TAG = "tag_STATE";

UwbTagDevice::UwbTagDevice(const std::vector<std::shared_ptr<UwbAnchorData>> & anchors,
                           const uint32_t rangingIntervalMs,
                           sensor::Sensor* latitudeSensor,
                           sensor::Sensor* longitudeSensor,
                           sensor::Sensor* locationErrorEstimateSensor)
: RX_BUF_LEN(std::max(ResponseMsg::FRAME_SIZE, FinalMsg::FRAME_SIZE)),
  RANGING_INTERVAL_MS(rangingIntervalMs)
{
    mAnchors = std::move(anchors);
    rx_buffer = new uint8_t(RX_BUF_LEN);
    mLatitudeSensor = latitudeSensor;
    mLongitudeSensor = longitudeSensor;
    mLocationErrorEstimateSensor = locationErrorEstimateSensor;
}

UwbTagDevice::~UwbTagDevice() {
    delete rx_buffer;
}

void UwbTagDevice::setup() {
    Dw3000Device::setup();

    setMyState(MYSTATE_PREPARE_SEND_INITIAL);
}

void UwbTagDevice::loop() {
    Dw3000Device::loop();

    do_ranging();
}

void UwbTagDevice::setMyState(const eMyState newState) {
    if (currState != newState) {
        // update state
        prevState = currState;
        currState = newState;
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_WARN
/* YAML must contain at least:
  logger:
    level: WARN
    logs:
      tag_STATE: WARN
*/
        const uint64_t nowMicros = micros();
        const char* newStateStr;
        int logLvl = ESPHOME_LOG_LEVEL_NONE;
        switch (newState) {
            case MYSTATE_UNKNOWN:                   newStateStr = "UNKNOWN"; logLvl = ESPHOME_LOG_LEVEL_ERROR; break;
            case MYSTATE_PREPARE_SEND_INITIAL:      newStateStr = "PREPARE_SEND_INITIAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_SENT_INITIAL:              newStateStr = "SENT_INITIAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_SEND_ERROR_INITIAL:        newStateStr = "SEND_ERROR_INITIAL"; logLvl = ESPHOME_LOG_LEVEL_WARN; break;
            case MYSTATE_WAIT_RECV_RESPONSE:        newStateStr = "WAIT_RECV_RESPONSE"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_FRAME_RESPONSE:      newStateStr = "RECVD_FRAME_RESPONSE"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_VALID_RESPONSE:      newStateStr = "RECVD_VALID_RESPONSE"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_INVALID_RESPONSE:    newStateStr = "RECVD_INVALID_RESPONSE"; logLvl = ESPHOME_LOG_LEVEL_WARN; break;
            case MYSTATE_SENT_FINAL:                newStateStr = "SENT_FINAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_SEND_ERROR_FINAL:          newStateStr = "SEND_ERROR_FINAL"; logLvl = ESPHOME_LOG_LEVEL_WARN; break;
            case MYSTATE_WAIT_RECV_FINAL:           newStateStr = "WAIT_RECV_FINAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_FRAME_FINAL:         newStateStr = "RECVD_FRAME_FINAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_VALID_FINAL:         newStateStr = "RECVD_VALID_FINAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_INVALID_FINAL:       newStateStr = "RECVD_INVALID_FINAL"; logLvl = ESPHOME_LOG_LEVEL_WARN; break;
            case MYSTATE_RANGING_DONE:              newStateStr = "RANGING_DONE"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_CALCULATE_LOCATION:        newStateStr = "CALCULATE_LOCATION"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            default:                                newStateStr = "?!?"; logLvl = ESPHOME_LOG_LEVEL_ERROR; break;
        }
        switch (logLvl) {
            case ESPHOME_LOG_LEVEL_INFO:
                ESP_LOGI(STATE_TAG, "%" PRIu64 " us: %s", nowMicros, newStateStr);
                break;
            case ESPHOME_LOG_LEVEL_WARN:
                ESP_LOGW(STATE_TAG, "%" PRIu64 " us: %s", nowMicros, newStateStr);
                break;
            case ESPHOME_LOG_LEVEL_ERROR:
                ESP_LOGE(STATE_TAG, "%" PRIu64 " us: %s", nowMicros, newStateStr);
                break;
            default:
                break;
        }
#endif
    }
}

void UwbTagDevice::do_ranging() {
    if (mAnchors.empty()) {
        return; // no anchors configured, nothing to do
    }

    switch (currState) {
        case MYSTATE_PREPARE_SEND_INITIAL:       prepareSendInitial(); break;
        case MYSTATE_SENT_INITIAL:               sentInitial(); break;
        case MYSTATE_SEND_ERROR_INITIAL:         sendInitialError(); break;
        case MYSTATE_WAIT_RECV_RESPONSE:         waitRecvResponse(); break;
        case MYSTATE_RECVD_FRAME_RESPONSE:       recvdFrameResponse(); break;
        case MYSTATE_RECVD_VALID_RESPONSE:       recvdValidResponse(); break;
        case MYSTATE_RECVD_INVALID_RESPONSE:     recvdInvalidResponse() ; break;
        case MYSTATE_SENT_FINAL:                 sentFinal(); break;
        case MYSTATE_SEND_ERROR_FINAL:           sendErrorFinal(); break;
        case MYSTATE_WAIT_RECV_FINAL:            waitRecvFinal(); break;
        case MYSTATE_RECVD_FRAME_FINAL:          recvdFrameFinal(); break;
        case MYSTATE_RECVD_VALID_FINAL:          recvdValidFinal(); break; // should not occur because direct call
        case MYSTATE_RECVD_INVALID_FINAL:        recvdInvalidFinal(); break;
        case MYSTATE_RANGING_DONE:               rangingDone(false); break; // // should not occur because direct call
        case MYSTATE_CALCULATE_LOCATION:         calculateLocation(); break;
        default:
            ESP_LOGE(TAG, "unhandled state %d", currState);
            // reset state machine
            setMyState(MYSTATE_PREPARE_SEND_INITIAL);
            break;
    }
}

void UwbTagDevice::prepareSendInitial() {

    const uint32_t now = millis();
    if (now - mLastInitialSentMillis < RANGING_INTERVAL_MS) {
        // too early, check in next loop() again
        return;
    }

    // find next anchor to do ranging with. There is at least 1 anchor, otherwise we would not be here
    if (mCurrentAnchorIndex < (mAnchors.size() -1)) {
        mCurrentAnchorIndex++;
    } else {
        mCurrentAnchorIndex = 0;
    }

    /* Set expected Response's delay and timeout. */
    dwt_setrxaftertxdelay(INITIAL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    /* Prepare Initial frame. */
    mInitialFrame.resetToDefault();
    mInitialFrame.setSequenceNumber(getNextTxSequenceNumberAndIncrease());
    const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
    mInitialFrame.setTargetId(anchorId);
    mInitialFrame.setSourceId(getDeviceId());
    if (mInitialFrame.isValid()) {
        ESP_LOGV(TAG, "Tag 0x%02X: Initiating to Anchor 0x%02X", getDeviceId(), anchorId);

        uint8_t* tx_buffer = mInitialFrame.getBytes().data();
        /* Clear Transmit Frame Sent. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        /* Write frame data to DW IC and prepare transmission. */
        if (dwt_writetxdata(InitialMsg::FRAME_SIZE, tx_buffer, 0 /*zero offset*/) != DWT_SUCCESS) {
            mTxErrorCount++;
            ESP_LOGE(TAG, "Write TX failed (total TX errors %" PRIu32 "x)", mTxErrorCount);
            setMyState(MYSTATE_SEND_ERROR_INITIAL);
            return;
        }
        dwt_writetxfctrl(InitialMsg::FRAME_SIZE, 0 /*zero offset*/, 1 /*=ranging*/);
        /* Start transmission, indicating that a response is expected so that reception is enabled automatically
           after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed. */
        if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) == DWT_SUCCESS) {
            mLastInitialSentMillis = now; // set time of when this function was entered, ensures a more acurrate RANGING_INTERVAL_MS
            mHighFreqLoopRequester.start(); // must be able to receive Response within microseconds
            setMyState(MYSTATE_SENT_INITIAL);
        } else {
            mTxErrorCount++;
            ESP_LOGE(TAG, "TX_IMMEDIATE failed (total TX errors %" PRIu32 "x)", mTxErrorCount);
            setMyState(MYSTATE_SEND_ERROR_INITIAL);
        }
    } else {
        ESP_LOGE(TAG, "Initial frame invalid");
    }
}

void UwbTagDevice::sentInitial() {
    /* remember when MYSTATE_WAIT_RECV_RESPONSE was entered for timeout calculation. */
    mEnteredWaitRecvResponseMicros = micros();
    /* Retrieve Initial transmission timestamp. */
    mInitial_tx_ts = get_tx_timestamp_u64();
    setMyState(MYSTATE_WAIT_RECV_RESPONSE);
}

void UwbTagDevice::waitRecvResponse() {
    /* Check for timeout */
    const uint32_t startedPollLoopMicros = micros();
    if ((startedPollLoopMicros - mEnteredWaitRecvResponseMicros) >= (WAIT_RX_TIMEOUT_MS * 1000UL)) {
        // next ranging loop
        //ESP_LOGW(TAG, "Timeout awaiting Response after %" PRIu32 " ms", WAIT_RX_TIMEOUT_MS);
        setMyState(MYSTATE_PREPARE_SEND_INITIAL);
        return;
    }
    /* Poll for reception of expected Response frame or error/timeout. */
    uint32_t status_reg;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        if ((micros() - startedPollLoopMicros) >= (MAX_POLL_DURATION_MS * 1000UL)) {
            // abort polling for now until next loop()
            return;
        }
    }
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        setMyState(MYSTATE_RECVD_FRAME_RESPONSE);

        /* Clear good RX frame event and TX frame sent in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        // immediate call instead of state change
        recvdFrameResponse();

    } else {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        const uint32_t waitMicros = micros() - mEnteredWaitRecvResponseMicros;
        const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
        if (status_reg & SYS_STATUS_ALL_RX_TO) {
            if ((status_reg & SYS_STATUS_RXFTO_BIT_MASK) == SYS_STATUS_RXFTO_BIT_MASK)
                ESP_LOGW(TAG, "0x%02X: waitRecvResponse RX Frame Wait timeout after %" PRIu32 " us", anchorId, waitMicros);
            if ((status_reg & SYS_STATUS_RXPTO_BIT_MASK) == SYS_STATUS_RXPTO_BIT_MASK)
                ESP_LOGW(TAG, "0x%02X: waitRecvResponse RX Preamble Detection timeout after %" PRIu32 " us", anchorId, waitMicros);
        } else if (status_reg & SYS_STATUS_ALL_RX_ERR) {
            ESP_LOGE(TAG, "0x%02X: waitRecvResponse RX error after %" PRIu32 " us", anchorId, waitMicros);
            rangingDone(false);
        } else {
            ESP_LOGE(TAG, "0x%02X: waitRecvResponse status_reg=0x%08x after %" PRIu32 " us", anchorId, status_reg, waitMicros);
            rangingDone(false);
        }
    }
}

void UwbTagDevice::recvdFrameResponse() {
    /* A frame has been received, read it into the local buffer. */
    const uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    } else {
        const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
        ESP_LOGW(TAG, "0x%02X: recvdFrameResponse frame bytes %" PRIu16 " exceeds RX_BUF_LEN %zu", anchorId, frame_len, RX_BUF_LEN);
        rangingDone(false);
        return;
    }
    /* Check that the frame is the expected Response and targeted to this device. */
    const auto responseMsg = std::make_shared<ResponseMsg>(rx_buffer, frame_len);
    bool proceed = responseMsg->isValid();
    uint8_t anchorId;
    if (proceed) {
        const uint8_t targetId = responseMsg->getTargetId();
        if (targetId != getDeviceId()) {
            proceed = false;
        }
        anchorId = responseMsg->getSourceId();
        // skipping the check for anchorId against current mCurrentAnchorIndex to save processing time
    }
    if (proceed) {
        /* Yes, it is the frame we are expecting. */
        setMyState(MYSTATE_RECVD_VALID_RESPONSE);

        /* Retrieve reception timestamp of this incoming frame. */
        const uint64_t response_rx_ts = get_rx_timestamp_u64();

        /* Compute Final message delayed transmission time. */
        const uint64_t final_tx_time =
            ((response_rx_ts + ((uint64_t)RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME))  & 0x00FFFFFFFFFFFFFFUL) >> 8;
        dwt_setdelayedtrxtime((uint32_t)final_tx_time);

        /* Set expected Final's delay and timeout. */
        dwt_setrxaftertxdelay(FINAL_TX_TO_FINAL_RESPONSE_RX_DLY_UUS);
        dwt_setrxtimeout(FINAL_RESPONSE_RX_TIMEOUT_UUS);
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);

        /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
        const uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        /* Write all timestamps in the Final message. */
        mFinalFrame.resetToDefault();
        mFinalFrame.setSequenceNumber(Dw3000Device::getNextTxSequenceNumberAndIncrease());
        mFinalFrame.setTimestamps((uint32_t)mInitial_tx_ts,
                                  (uint32_t)response_rx_ts,
                                  (uint32_t)final_tx_ts);
        mFinalFrame.setTargetId(anchorId);
        mFinalFrame.setSourceId(getDeviceId());
        /* Write and send Final message. */
        // don't check mFinalFrame.isValid() in order to save processing time
        uint8_t* txbuffer = mFinalFrame.getBytes().data();
        dwt_writetxdata(FinalMsg::FRAME_SIZE, txbuffer, 0 /*zero offset*/);
        dwt_writetxfctrl(FinalMsg::FRAME_SIZE, 0 /*zero offset*/, 1 /*=ranging */);

        const uint32_t systime = dwt_readsystimestamphi32();
        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
#ifdef USE_DS_TWR_SYNCRONOUS
        mResponse_rx_ts = response_rx_ts;
        if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) == DWT_SUCCESS) {
#else
        if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {
#endif
            setMyState(MYSTATE_SENT_FINAL);
            ESP_LOGV(TAG, "Final sent Ok: systime=%" PRIu32 ", final_tx_time=%" PRIu64 ", diff=%" PRId32 ,
                    systime, final_tx_time, diff);
        } else {
            const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
            mTxErrorCount++;
            ESP_LOGE(TAG, "0x%02X: Final TX_DELAYED failed (total %" PRIu32 "x)", anchorId, mTxErrorCount);
            // dwt_starttx(DWT_START_TX_DELAYED) likely failed due to SYS_STATUS HPDWARN bit
            const int32_t diff = (uint32_t)final_tx_time - systime; // diff should be positive in good case
            const uint64_t response_rx_time = (response_rx_ts & 0x00FFFFFFFFFFFFFFUL) >> 8;
            ESP_LOGW(TAG, "  systime=%" PRIu32 ", final_tx_time=%" PRIu64 ", diff=%" PRId32 ,
                systime, final_tx_time, diff);
            setMyState(MYSTATE_SEND_ERROR_FINAL);
        }

        /* after(!) trying to send Final frame, read function code and data from Response frame
           -- reserved for future use --
        */
        uint8_t data[ResponseMsg::RESPONSE_DATA_SIZE];
        std::size_t actualdataSize;
        uint8_t fctCode;
        if (responseMsg->getFunctionCodeAndData(&fctCode, data, ResponseMsg::RESPONSE_DATA_SIZE, &actualdataSize)) {
            if (fctCode == ResponseMsg::RESPONSE_FCT_CODE_RANGING) {
                const uint16_t dummyData = (uint16_t)(data[0] << 8) + (uint16_t) data[1];
            } else {
                const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
                ESP_LOGE(TAG, "0x%02X: getFunctionCodeAndData from Response frame failed", anchorId);
            }
        } else {
            const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
            ESP_LOGE(TAG, "0x%02X: getFunctionCodeAndData from Response frame failed", anchorId);
        }
    } else {
        setMyState(MYSTATE_RECVD_INVALID_RESPONSE);
    }
}

void UwbTagDevice::sendInitialError() {
    rangingDone(false);
}

void UwbTagDevice::waitRecvFinal() {
    /* Check for timeout */
    const uint32_t startedPollLoopMicros = micros();
    if ((startedPollLoopMicros - mEnteredWaitRecvFinalMicros) >= (WAIT_RX_TIMEOUT_MS * 1000U)) {
        rangingDone(false);
    }
    /* Poll for reception of expected Final response frame or error/timeout. */
    uint32_t status_reg;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        if ((micros() - startedPollLoopMicros) >= (MAX_POLL_DURATION_MS * 1000U)) {
            // abort polling for now until next loop()
            return;
        }
    }
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        setMyState(MYSTATE_RECVD_FRAME_FINAL);

        /* Clear good RX frame event and TX frame sent in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        // immediate call instead of state change
        recvdFrameFinal();

    } else {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        const uint32_t waitMicros = micros() - mEnteredWaitRecvFinalMicros;
        const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
        if (status_reg & SYS_STATUS_ALL_RX_TO) {
            if ((status_reg & SYS_STATUS_RXFTO_BIT_MASK) == SYS_STATUS_RXFTO_BIT_MASK)
                ESP_LOGW(TAG, "0x%02X: waitRecvFinal RX Frame Wait timeout after %" PRIu32 " us", anchorId, waitMicros);
            if ((status_reg & SYS_STATUS_RXPTO_BIT_MASK) == SYS_STATUS_RXPTO_BIT_MASK)
                ESP_LOGW(TAG, "0x%02X: waitRecvFinal RX Preamble Detection timeout after %" PRIu32 " us", anchorId, waitMicros);
        } else if (status_reg & SYS_STATUS_ALL_RX_ERR) {
            ESP_LOGE(TAG, "0x%02X: waitRecvFinal RX error after %" PRIu32 " us", anchorId, waitMicros);
            rangingDone(false);
        } else {
            ESP_LOGE(TAG, "0x%02X: waitRecvFinal status_reg=0x%08x after %" PRIu32 " us", anchorId, status_reg, waitMicros);
            rangingDone(false);
        }
    }
}

void UwbTagDevice::recvdFrameFinal() {
    /* A frame has been received, read it into the local buffer. */
    const uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    } else {
        const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
        ESP_LOGW(TAG, "0x%02X: recvdFrameFinal frame bytes %" PRIu16 " exceeds RX_BUF_LEN %zu", anchorId, frame_len, RX_BUF_LEN);
        rangingDone(false);
        return;
    }
    /* Check that the frame is the expected Final response and targeted to this device. */
    const auto finalMsg = std::make_shared<FinalMsg>(rx_buffer, frame_len);
    bool proceed = finalMsg->isValid();
    uint8_t anchorId;
    const uint8_t myDeviceId = getDeviceId();
    if (proceed) {
        const uint8_t targetId = finalMsg->getTargetId();
        if (targetId != myDeviceId) {
            proceed = false;
        }
        anchorId = finalMsg->getSourceId();
        // skipping the check for anchorId against current mCurrentAnchorIndex to save processing time
    }
    if (proceed) {
        setMyState(MYSTATE_RECVD_VALID_FINAL);

        /* Retrieve Final transmission and Final response reception timestamps. */
        const uint64_t final_tx_ts = get_tx_timestamp_u64();
        const uint64_t final_response_rx_ts = get_rx_timestamp_u64();

        /* Get timestamps embedded in the Final response message. */
        uint32_t response_tx_ts, final_rx_ts, final_response_tx_ts;
        finalMsg->getTimestamps(&response_tx_ts, &final_rx_ts, &final_response_tx_ts);

        /* Compute time of flight. */
        const double Ra = (double)(final_rx_ts - response_tx_ts);
        const double Rb = (double)(final_response_rx_ts - final_tx_ts);
        const double Da = (double)(final_response_tx_ts - final_rx_ts);
        const double Db = (double)(final_tx_ts - mResponse_rx_ts);
        const int64_t tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
        const double tof = tof_dtu * DWT_TIME_UNITS;
        const double distance = tof * SPEED_OF_LIGHT;
        const uint32_t distanceUpdated = millis();

        /* Display computed distance. */
        ESP_LOGW(TAG, "DIST anchor 0x%.2X: %.2fm", anchorId, distance);
        if (mCurrentAnchorIndex > -1) {
            (mAnchors[mCurrentAnchorIndex])->setDistance(distance);
        } else {
            ESP_LOGE(TAG, "mCurrentAnchorIndex=%i", mCurrentAnchorIndex);
        }

        rangingDone(true);

    } else {
        setMyState(MYSTATE_RECVD_INVALID_FINAL);
    }
}

void UwbTagDevice::recvdValidResponse() {
    // should not come here
    ESP_LOGE(TAG, "entered recvdValidResponse() unexpectedly");
    rangingDone(false);
}

void UwbTagDevice::recvdInvalidResponse() {
    rangingDone(false);
}

void UwbTagDevice::sentFinal() {
#ifdef USE_DS_TWR_SYNCRONOUS
    /* remember when MYSTATE_WAIT_RECV_FINAL was entered for timeout calculation. */
    mEnteredWaitRecvFinalMicros = micros();
    setMyState(MYSTATE_WAIT_RECV_FINAL);
#else
    setMyState(MYSTATE_PREPARE_SEND_INITIAL);
#endif
}

void UwbTagDevice::sendErrorFinal() {
    rangingDone(false);
}

void UwbTagDevice::recvdValidFinal() {
    // should not come here
    ESP_LOGE(TAG, "entered recvdValidFinal() unexpectedly");
    rangingDone(false);
}

void UwbTagDevice::recvdInvalidFinal() {
    rangingDone(false);
}

void UwbTagDevice::rangingDone(bool success) {
     // if running with high-frequency loop(), go back to normal frequency
    mHighFreqLoopRequester.stop();

    if (success) {
        setMyState(MYSTATE_CALCULATE_LOCATION);
    } else {
        setMyState(MYSTATE_PREPARE_SEND_INITIAL);
    }
}

void UwbTagDevice::calculateLocation() {
    // collect all distances to anchors
    std::vector<AnchorPositionTagDistance> anchorPositionAndTagDistances;
    for(const auto anchor: mAnchors) {
        AnchorPositionTagDistance anchorPosAndTagDist;
        anchorPosAndTagDist.anchorId = anchor->getId();
        anchorPosAndTagDist.anchorPosition.latitude =  anchor->getLatitude();
        anchorPosAndTagDist.anchorPosition.longitude = anchor->getLongitude();
        anchorPosAndTagDist.tagDistance = anchor->getDistance(nullptr); // no need for the age of the distance
        if (Location::isValid(anchorPosAndTagDist)) {
            anchorPositionAndTagDistances.push_back(anchorPosAndTagDist);
        }
    }

    // calculate location and its error estimate
    LatLong tagPosition;
    double errorEstimateMeters;
    const uint32_t startMicros = micros();
    CalcResult res = Location::calculatePosition(anchorPositionAndTagDistances, tagPosition, errorEstimateMeters);
    const uint32_t endMicros = micros();
    if (CALC_OK == res) {
        if (Location::isValid(tagPosition)) {
            ESP_LOGW(TAG, "0x%02x: position %.7f,%.7f errEst %.2fm duration %" PRIu32 "us",
                getDeviceId(), tagPosition.latitude, tagPosition.longitude, errorEstimateMeters, endMicros-startMicros);

            // report location and error estimate
            if (mLatitudeSensor != nullptr) {
                mLatitudeSensor->publish_state(tagPosition.latitude);
            }
            if (mLongitudeSensor != nullptr) {
                mLongitudeSensor->publish_state(tagPosition.longitude);
            }
            if (mLocationErrorEstimateSensor != nullptr) {
                mLocationErrorEstimateSensor->publish_state(errorEstimateMeters);
            }
        } else {
            ESP_LOGW(TAG, "calculatePosition result invalid: 0x%02x: position %.7f,%.7f errEst %.2fm duration %" PRIu32 "us",
                getDeviceId(), tagPosition.latitude, tagPosition.longitude, errorEstimateMeters, endMicros-startMicros);
        }
    } else {
        ESP_LOGW(TAG, "0x%02x: calculatePosition failed: %i", getDeviceId(), res);
    }

    // next ranging cycle
    setMyState(MYSTATE_PREPARE_SEND_INITIAL);
}

}  // namespace uwb
}  // namespace esphome

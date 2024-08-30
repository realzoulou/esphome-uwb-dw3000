#include "UwbAnchorDevice.h"
#include "InitialMsg.h"
#include "ResponseMsg.h"
#include "FinalMsg.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/log.h"

#include "dw3000.h"

namespace esphome {
namespace uwb {

const char* UwbAnchorDevice::TAG = "anchor";
const char* UwbAnchorDevice::STATE_TAG = "anchor_STATE";

UwbAnchorDevice::UwbAnchorDevice()
: RX_BUF_LEN(std::max(InitialMsg::FRAME_SIZE, FinalMsg::FRAME_SIZE)) {
    rx_buffer = new uint8_t(RX_BUF_LEN);
}
UwbAnchorDevice::~UwbAnchorDevice() {
    delete rx_buffer;
}

void UwbAnchorDevice::setup() {
    Dw3000Device::setup();

    setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
}

void UwbAnchorDevice::loop() {
    Dw3000Device::loop();

    do_ranging();
}

void UwbAnchorDevice::setMyState(const eMyState newState) {
    if (currState != newState) {
        // update state
        prevState = currState;
        currState = newState;
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_INFO
/* YAML must contain at least:
  logger:
    level: INFO
    logs:
      anchor_STATE: INFO
*/
        const uint64_t nowMicros = micros();
        const char* newStateStr;
        int logLvl = ESPHOME_LOG_LEVEL_NONE;
        switch (newState) {
            case MYSTATE_UNKNOWN:                         newStateStr = "UNKNOWN"; logLvl = ESPHOME_LOG_LEVEL_ERROR; break;
            case MYSTATE_PREPARE_WAIT_RECV_INITIAL:       newStateStr = "PREPARE_WAIT_RECV_INITIAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_WAIT_RECV_INITIAL:               newStateStr = "WAIT_RECV_INITIAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_FRAME_INITIAL:             newStateStr = "RECVD_FRAME_INITIAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_FRAME_VALID_INITIAL:       newStateStr = "RECVD_FRAME_VALID_INITIAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_FRAME_INVALID_INITIAL:     newStateStr = "RECVD_FRAME_INVALID_INITIAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_SENT_RESPONSE:                   newStateStr = "SENT_RESPONSE"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_SEND_ERROR:                      newStateStr = "SEND_ERROR"; logLvl = ESPHOME_LOG_LEVEL_WARN; break;
            case MYSTATE_WAIT_RECV_FINAL:                 newStateStr = "WAIT_RECV_FINAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_FRAME_FINAL:               newStateStr = "RECVD_FRAME_FINAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_FRAME_VALID_FINAL:         newStateStr = "RECVD_FRAME_VALID_FINAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_FRAME_INVALID_FINAL:       newStateStr = "RECVD_FRAME_INVALID_FINAL"; logLvl = ESPHOME_LOG_LEVEL_WARN; break;
            default:                                      newStateStr = "?!?"; logLvl = ESPHOME_LOG_LEVEL_ERROR; break;
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

void UwbAnchorDevice::do_ranging() {
    switch (currState) {
        case MYSTATE_PREPARE_WAIT_RECV_INITIAL:    prepareWaitRecvInitial(); break;
        case MYSTATE_WAIT_RECV_INITIAL:            waitRecvInitial(); break;
        case MYSTATE_RECVD_FRAME_INITIAL:          recvdFrameInitial(); break;
        case MYSTATE_RECVD_FRAME_VALID_INITIAL:    recvdFrameValidInitial(); break; // should not occur because direct call
        case MYSTATE_RECVD_FRAME_INVALID_INITIAL:  recvdFrameInvalidInitial(); break;
        case MYSTATE_SENT_RESPONSE:                sentResponse(); break; // should not occur because direct call
        case MYSTATE_SEND_ERROR:                   sendError(); break;
        case MYSTATE_WAIT_RECV_FINAL:              waitRecvFinal(); break;
        case MYSTATE_RECVD_FRAME_FINAL:            recvdFrameFinal(); break;
        case MYSTATE_RECVD_FRAME_VALID_FINAL:      recvdFrameValidFinal(); break; // should not occur because direct call
        case MYSTATE_RECVD_FRAME_INVALID_FINAL:    recvdFrameInvalidFinal(); break;
        default:
            ESP_LOGE(TAG, "unhandled state %d", currState);
            // reset state machine
            setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
            break;
    }
}

void UwbAnchorDevice::prepareWaitRecvInitial() {
    mHighFreqLoopRequester.stop();
    /* Clear reception timeout to start next ranging process. */
    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout(0);

    /* Activate reception immediately. */
    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) == DWT_ERROR) {
        const uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        ESP_LOGE(TAG, "dwt_rxenable failed: SYS_STATUS: 0x%08x", status_reg);
    }

    setMyState(MYSTATE_WAIT_RECV_INITIAL);
}

void UwbAnchorDevice::waitRecvInitial() {
    const uint64_t startedPollLoopMicros = micros();
    /* Poll for reception of a "initial" frame or error/timeout. */
    uint32_t status_reg;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        if (micros() - startedPollLoopMicros >= (MAX_POLL_DURATION_MS * 1000U)) {
            // abort polling for now until next loop
            mHighFreqLoopRequester.stop();
            return;
        }
    }
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        setMyState(MYSTATE_RECVD_FRAME_INITIAL);

        mHighFreqLoopRequester.start();

        /* Clear good RX frame event in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        /* Retrieve poll reception timestamp. */
        mInitial_rx_ts = get_rx_timestamp_u64();

        // immediate call instead of state change
        recvdFrameInitial();
    }  else {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
}

void UwbAnchorDevice::recvdFrameInitial() {
    /* A frame has been received, read it into the local buffer. */
    const uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    } else {
        ESP_LOGW(TAG, "recvd frame bytes %" PRIu16 " exceeds RX_BUF_LEN %zu", frame_len, RX_BUF_LEN);
        setMyState(MYSTATE_RECVD_FRAME_INVALID_INITIAL);
        return;
    }
    /* Check that the frame is an Initial frame and targeted to this device. */
    const auto initialMsg = std::make_shared<InitialMsg>(rx_buffer, frame_len);
    bool proceed = initialMsg->isValid();
    if (proceed) {
        const uint8_t targetId = initialMsg->getTargetId();
        if (targetId != getDeviceId()) {
            // silently ignore
            proceed = false;
            setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
            return;
        } else {
            // remember the source ID as the tag's device ID
            mCurrentTagId = initialMsg->getSourceId();
        }
    }
    if (proceed) {
        /* Yes, it is the frame we are expecting. */
        setMyState(MYSTATE_RECVD_FRAME_VALID_INITIAL);

        /* Set send time for response. */
        const uint64_t response_tx_time =
            ((mInitial_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) & 0x00FFFFFFFFFFFFFFUL) >> 8;
        dwt_setdelayedtrxtime((uint32_t)response_tx_time);

        /* Set expected delay and timeout for final message reception. */
        dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
        dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
        /* Set preamble timeout for expected frames. */
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);

        /* Write and send the response message. */
        mResponseFrame.resetToDefault();
        mResponseFrame.setSequenceNumber(Dw3000Device::getNextTxSequenceNumberAndIncrease());
        mResponseFrame.setTargetId(mCurrentTagId);
        mResponseFrame.setSourceId(getDeviceId());
        /* Add the previously calculated distance in cm, 16-bit BigEndian */
        /* TODO support for >1 tag */
        const uint16_t prev_dist_cm = (uint16_t) (getLastDistance(nullptr) * 100.0);
        uint8_t respMsgData[2];
        respMsgData[0] = (uint8_t) ((prev_dist_cm & 0xFF00) >> 8);
        respMsgData[1] = (uint8_t) (prev_dist_cm & 0x00FF);
        if (!mResponseFrame.setFunctionCodeAndData(ResponseMsg::RESPONSE_FCT_CODE_RANGING, respMsgData, 2)) {
            ESP_LOGE(TAG, "setFunctionCodeAndData failed");
            setMyState(MYSTATE_SEND_ERROR);
            return;
        }
        // not checking mResponseFrame.isValid() in order to save processing time
        uint8_t* txbuffer = mResponseFrame.getBytes().data();
        const std::size_t txBufferSize = ResponseMsg::FRAME_SIZE;
        dwt_writetxdata(txBufferSize, txbuffer, 0);                   /* Zero offset in TX buffer. */
        dwt_writetxfctrl(txBufferSize, 0, 1 /* 1=ranging */);         /* Zero offset in TX buffer, ranging. */

        const uint32_t systime = (uint64_t) dwt_readsystimestamphi32();
        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
        if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) == DWT_ERROR) {
            mTxErrorCount++;
            ESP_LOGE(TAG, "Response TX_DELAYED failed (total %" PRIu32 "x)", mTxErrorCount);
            // dwt_starttx(DWT_START_TX_DELAYED) likely failed due to SYS_STATUS HPDWARN bit
            const int32_t diff = (uint32_t)response_tx_time - systime; // diff should be positive in good case
            ESP_LOGW(TAG, "systime=%" PRIu32 ", response_tx_time=%" PRIu64 ", diff=%" PRId32 ,
                systime, response_tx_time, diff);
            setMyState(MYSTATE_SEND_ERROR);
        } else {
            setMyState(MYSTATE_SENT_RESPONSE);

            mEnteredWaitRecvFinalMicros = micros();
            setMyState(MYSTATE_WAIT_RECV_FINAL);
        }
    } else {
        setMyState(MYSTATE_RECVD_FRAME_INVALID_INITIAL);
    }
}

void UwbAnchorDevice::waitRecvFinal() {
    /* Check for timeout */
    const uint64_t startedPollLoopMicros = micros();
    if (startedPollLoopMicros - mEnteredWaitRecvFinalMicros >= (WAIT_FINAL_RX_TIMEOUT_MS * 1000U)) {
        /* Abort this ranging attempt */
        ESP_LOGW(TAG, "Timeout awaiting Final after %" PRIu32 " ms", WAIT_FINAL_RX_TIMEOUT_MS);
        setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
        return;
    }
    /* Poll for reception of expected "final" frame or error/timeout. */
    uint32_t status_reg;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        if (micros() - startedPollLoopMicros >= (MAX_POLL_DURATION_MS * 1000U)) {
            // abort polling for now until next loop
            return;
        }
    }
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        /* Clear good RX frame event in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        setMyState(MYSTATE_RECVD_FRAME_FINAL);

        // immediate call instead of state change
        recvdFrameFinal();
    }  else {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }
}

void UwbAnchorDevice::recvdFrameFinal() {
    /* A frame has been received, read it into the local buffer. */
    const uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    }
    /* Check that the frame is a Final message sent by inititating tag device and targeted to this device. */
    const auto finalMsg = std::make_shared<FinalMsg>(rx_buffer, frame_len);
    bool proceed = finalMsg->isValid();
    if (proceed) {
        const uint8_t targetId = finalMsg->getTargetId();
        if (targetId != getDeviceId()) {
            // silently ignore
            proceed = false;
            setMyState(MYSTATE_WAIT_RECV_FINAL); // not MYSTATE_RECVD_FRAME_INVALID_FINAL
            return;
        }
    }
    if (proceed) {

        setMyState(MYSTATE_RECVD_FRAME_VALID_FINAL);

        /* Retrieve response transmission and final reception timestamps. */
        const uint64_t resp_tx_ts = get_tx_timestamp_u64();
        const uint64_t final_rx_ts = get_rx_timestamp_u64();

        /* Get timestamps embedded in the final message. */
        uint32_t initial_tx_ts, resp_rx_ts, final_tx_ts;
        finalMsg->getTimestamps(&initial_tx_ts, &resp_rx_ts, &final_tx_ts);

        /* Compute time of flight. */
        const double Ra = (double)(resp_rx_ts - initial_tx_ts);
        const double Rb = (double)(final_rx_ts - resp_tx_ts);
        const double Da = (double)(final_tx_ts - resp_rx_ts);
        const double Db = (double)(resp_tx_ts - mInitial_rx_ts);
        const int64_t tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
        const double tof = tof_dtu * DWT_TIME_UNITS;
        const double distance = tof * SPEED_OF_LIGHT;

        /* Display computed distance. */
        ESP_LOGI(TAG, "DIST: %.2f m", distance);
        setLastDistance(distance); // TODO: support >1 tag

        /* Next ranging cycle. */
        setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);

    } else {
        setMyState(MYSTATE_RECVD_FRAME_INVALID_FINAL);
    }
}

void UwbAnchorDevice::recvdFrameValidInitial() {
    // if we come here, then the state machine got stuck
    // reset state machine
    setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
}

void UwbAnchorDevice::recvdFrameInvalidInitial() {
    // reset state machine
    setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
}

void UwbAnchorDevice::sentResponse() {
    // if we come here, then the state machine got stuck
    // reset state machine
    setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
}

void UwbAnchorDevice::sendError() {
    // reset state machine
    setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
}

void UwbAnchorDevice::recvdFrameValidFinal() {
    // if we come here, then the state machine got stuck
    // reset state machine
    setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
}

void UwbAnchorDevice::recvdFrameInvalidFinal() {
    // reset state machine
    setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
}

double UwbAnchorDevice::getLastDistance(uint32_t* timeMillis) const {
    if (timeMillis != nullptr) {
        *timeMillis = mLastDistanceUpdatedMs;
    }
    return mLastDistance;
}

void UwbAnchorDevice::setLastDistance(const double distance) {
    /* Is this new distance really different to old one ? threshold is 1 cm. */
    if (std::fabs(distance - mLastDistance) > 0.01) {
        mLastDistance = distance;
        mLastDistanceUpdatedMs = millis();

        if (mListener != nullptr) {
            mListener->onDistanceUpdated(mLastDistance, mLastDistanceUpdatedMs);
        }
    }
}

}  // namespace uwb
}  // namespace esphome

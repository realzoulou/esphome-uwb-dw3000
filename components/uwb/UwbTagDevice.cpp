#include "UwbTagDevice.h"
#include "InitialMsg.h"
#include "ResponseMsg.h"
#include "FinalMsg.h"

#include "esphome/core/log.h"

#include "dw3000.h"

#define TX_ANT_DLY (16385)

namespace esphome {
namespace uwb {

const char* UwbTagDevice::TAG = "tag";
const char* UwbTagDevice::STATE_TAG = "tag_STATE";

UwbTagDevice::UwbTagDevice() {
    rx_buffer = new uint8_t(RX_BUF_LEN);
}

UwbTagDevice::~UwbTagDevice() {
    delete rx_buffer;
}

void UwbTagDevice::setup() {
    Dw3000Device::setup();

    /* Set expected Response's delay and timeout. Can be set here once for all. */
    dwt_setrxaftertxdelay(INITIAL_TX_TO_RESP_RX_DLY_UUS);
    
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    //set_resp_rx_timeout(RESP_RX_TIMEOUT_UUS, getConfig());

    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

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
            case MYSTATE_RECVD_FRAME:               newStateStr = "RECVD_FRAME"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_VALID_RESPONSE:      newStateStr = "RECVD_VALID_RESPONSE"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_INVALID_RESPONSE:    newStateStr = "RECVD_INVALID_RESPONSE"; logLvl = ESPHOME_LOG_LEVEL_WARN; break;
            case MYSTATE_SENT_FINAL:                newStateStr = "SENT_FINAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_SEND_ERROR_FINAL:          newStateStr = "SEND_ERROR_FINAL"; logLvl = ESPHOME_LOG_LEVEL_WARN; break;
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
    switch (currState) {
        case MYSTATE_PREPARE_SEND_INITIAL:       prepareSendInitial(); break;
        case MYSTATE_SENT_INITIAL:               sentInitial(); break;
        case MYSTATE_SEND_ERROR_INITIAL:         sendInitialError(); break;
        case MYSTATE_WAIT_RECV_RESPONSE:         waitRecvResponse(); break;
        case MYSTATE_RECVD_FRAME:                recvdFrameResponse(); break;
        case MYSTATE_RECVD_VALID_RESPONSE:       recvdValidResponse(); break;
        case MYSTATE_RECVD_INVALID_RESPONSE:     recvdInvalidResponse() ; break;
        case MYSTATE_SENT_FINAL:                 sentFinal(); break;
        case MYSTATE_SEND_ERROR_FINAL:           sendErrorFinal(); break;
        default:
            ESP_LOGE(TAG, "unhandled state %d", currState);
            // reset state machine
            setMyState(MYSTATE_PREPARE_SEND_INITIAL);
            break;
    }
}

void UwbTagDevice::prepareSendInitial() {
    mHighFreqLoopRequester.stop();
    const uint32_t now = millis();
    if (now - mLastInitialSentMillis < RANGING_INTERVAL_MS) {
        // too early
        return;
    }

    mInitialFrame.resetToDefault();
    const uint8_t seqNo = getNextTxSequenceNumberAndIncrease();
    if (!mInitialFrame.setSequenceNumber(seqNo)) {
        ESP_LOGE(TAG, "Failed to set sequence number 0x%02x in Initial", seqNo);
        setMyState(MYSTATE_SEND_ERROR_INITIAL);
        return;
    }
    uint8_t* tx_buffer = mInitialFrame.getBytes().data();
    /* Clear Transmit Frame Sent. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    /* Write frame data to DW IC and prepare transmission. */
    if (dwt_writetxdata(InitialMsg::FRAME_SIZE, tx_buffer, 0 /*zero offset*/) != DWT_SUCCESS) {
        mTxErrorCount++;
        ESP_LOGE(TAG, "Write TX failed (total TX errors %ux)", mTxErrorCount);
        setMyState(MYSTATE_SEND_ERROR_INITIAL);
        return;
    }
    dwt_writetxfctrl(InitialMsg::FRAME_SIZE, 0 /*zero offset*/, 1 /*=ranging*/);
    /* Start transmission, indicating that a response is expected so that reception is enabled automatically
       after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed. */
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) == DWT_SUCCESS) {
        mLastInitialSentMillis = now; // set time of when this function was entered, ensures a more acurrate RANGING_INTERVAL_MS
        mHighFreqLoopRequester.start(); // must be able to receive Reponse within microseconds
        setMyState(MYSTATE_SENT_INITIAL);
    } else {
        mTxErrorCount++;
        ESP_LOGE(TAG, "TX_IMMEDIATE failed (total TX errors %ux)", mTxErrorCount);
        setMyState(MYSTATE_SEND_ERROR_INITIAL);
    }
}

void UwbTagDevice::sentInitial() {
    /* remember when MYSTATE_WAIT_RECV_RESPONSE was entered for timeout calculation. */
    mEnteredWaitRecvResponseMicros = micros();
    /* Retrieve Initial transmission timestamp. */
    mInitial_tx_ts = get_tx_timestamp_u64();
    setMyState(MYSTATE_WAIT_RECV_RESPONSE);
}

void UwbTagDevice::sendInitialError() {
    // reset state machine
    setMyState(MYSTATE_PREPARE_SEND_INITIAL);
}

void UwbTagDevice::waitRecvResponse() {
    /* Check for timeout */
    const uint32_t startedPollLoopMicros = micros();
    if ((startedPollLoopMicros - mEnteredWaitRecvResponseMicros) >= (WAIT_RESPONSE_RX_TIMEOUT_MS * 1000UL)) {
        // next ranging loop
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
        setMyState(MYSTATE_RECVD_FRAME);

        /* Clear good RX frame event and TX frame sent in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        // immediate call instead of state change
        recvdFrameResponse();

    } else {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        const uint32_t waitMicros = micros() - mEnteredWaitRecvResponseMicros;
        if (status_reg & SYS_STATUS_ALL_RX_TO) {
            ESP_LOGI(TAG, "waitRecvResponse RX timeout after %" PRIu32 " us", waitMicros);
        } else if (status_reg & SYS_STATUS_ALL_RX_ERR) {
            ESP_LOGI(TAG, "waitRecvResponse RX error after %" PRIu32 " us", waitMicros);
        } else {
            ESP_LOGI(TAG, "waitRecvResponse status_reg=0x%08x after %" PRIu32 " us", status_reg, waitMicros);
        }
    }
}

void UwbTagDevice::recvdFrameResponse() {
    /* A frame has been received, read it into the local buffer. */
    const uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    } else {
        ESP_LOGW(TAG, "recvd frame bytes %u exceeds RX_BUF_LEN %u", frame_len, RX_BUF_LEN);
        setMyState(MYSTATE_PREPARE_SEND_INITIAL);
        return;
    }
    /* Check that the frame is the expected Response */
    //const auto responseMsg = std::make_shared<ResponseMsg>(rx_buffer, frame_len); // TODO
    if (true /*responseMsg->isValid()*/) {
        setMyState(MYSTATE_RECVD_VALID_RESPONSE);

        /* Retrieve reception timestamp of this incoming frame. */
        const uint64_t response_rx_ts = get_rx_timestamp_u64();

        /* Compute final message transmission time. */
        const uint64_t final_tx_time =
            ((response_rx_ts + ((uint64_t)RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME))  & 0x00FFFFFFFFFFFFFFUL) >> 8;
        
        dwt_setdelayedtrxtime((uint32_t)final_tx_time);
        //set_delayed_rx_time((uint32_t)final_tx_time, getConfig());

        /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
        const uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        /* Write all timestamps in the Final message. */
        mFinalFrame.resetToDefault();
        mFinalFrame.setSequenceNumber(Dw3000Device::getNextTxSequenceNumberAndIncrease());
        mFinalFrame.setTimestamps((uint32_t)mInitial_tx_ts, 
                                  (uint32_t)response_rx_ts,
                                  (uint32_t)final_tx_ts);
        /* Write and send Final message. */
        // don't check isValid() in order to save processing time
        uint8_t* txbuffer = mFinalFrame.getBytes().data();
        const std::size_t txBufferSize = FinalMsg::FRAME_SIZE;
        if (dwt_writetxdata(txBufferSize, txbuffer, 0 /*zero offset*/) != DWT_SUCCESS) {
            mTxErrorCount++;
            ESP_LOGE(TAG, "Write TX failed (total TX errors %ux)", mTxErrorCount);
            setMyState(MYSTATE_SEND_ERROR_FINAL);
            return;
        }
        dwt_writetxfctrl(txBufferSize, 0 /*zero offset*/, 1 /*=ranging */);
        
        const uint32_t systime = dwt_readsystimestamphi32();
        const int32_t diff = (uint32_t)final_tx_time - systime; // diff should be positive in good case

        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
        if (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS) {
            setMyState(MYSTATE_SENT_FINAL);
            ESP_LOGV(TAG, "Final sent Ok: systime=%" PRIu32 ", final_tx_time=%" PRIu64 ", diff=%" PRId32 , 
                    systime, final_tx_time, diff);
        } else {
            mTxErrorCount++;
            ESP_LOGE(TAG, "TX_DELAYED failed (total TX errors %ux)", mTxErrorCount);
            if (diff < 0) {
                // if diff is negative then dwt_starttx(DWT_START_TX_DELAYED) likely failed due to SYS_STATUS HPDWARN bit
                const uint64_t response_rx_time = (response_rx_ts & 0x00FFFFFFFFFFFFFFUL) >> 8;
                ESP_LOGW(TAG, "Slow processing! systime=%" PRIu32 ", final_tx_time=%" PRIu64 ", diff=%" PRId32 ,
                    systime, final_tx_time, diff);
                ESP_LOGW(TAG, "response_rx_time=%" PRIu64 ", diff=%" PRId32, response_rx_time, ((uint32_t)response_rx_time - systime));
            }
            setMyState(MYSTATE_SEND_ERROR_FINAL);
        }
    } else {
        setMyState(MYSTATE_RECVD_INVALID_RESPONSE);
    }
}
void UwbTagDevice::recvdValidResponse() {
    // should not come here
    ESP_LOGW(TAG, "entered recvdValidResponse() unexpectedly");
    // reset statemachine
    setMyState(MYSTATE_PREPARE_SEND_INITIAL);
}

void UwbTagDevice::recvdInvalidResponse() {
    // reset statemachine
    setMyState(MYSTATE_PREPARE_SEND_INITIAL);
}

void UwbTagDevice::sentFinal() {
    // next ranging loop
    setMyState(MYSTATE_PREPARE_SEND_INITIAL);
}

void UwbTagDevice::sendErrorFinal() {
    // reset statemachine
    setMyState(MYSTATE_PREPARE_SEND_INITIAL);
}

}  // namespace uwb
}  // namespace esphome

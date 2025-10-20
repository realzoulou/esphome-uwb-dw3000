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

#include "UwbAnchorDevice.h"
#include "InitialMsg.h"
#include "ResponseMsg.h"
#include "FinalMsg.h"
#include "Location.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/log.h"

#include "dw3000.h"

/* compile switch to enable (1) or disable (0) using direct function calls instead of waiting for state change. */
#ifndef ANCHOR_USE_IMMEDIATE_CALLS
#define ANCHOR_USE_IMMEDIATE_CALLS 1 // ranging takes avg. 3506 microseconds with TAG_USE_IMMEDIATE_CALLS=1
//#define ANCHOR_USE_IMMEDIATE_CALLS 0 // ranging takes avg. 3550 microseconds with TAG_USE_IMMEDIATE_CALLS=1
#endif

namespace esphome {
namespace uwb {

const char* UwbAnchorDevice::TAG = "anchor";
const char* UwbAnchorDevice::STATE_TAG = "anchor_STATE";

UwbAnchorDevice::UwbAnchorDevice(const double latitude, const double longitude,
                                 sensor::Sensor* latitudeSensor,
                                 sensor::Sensor* longitudeSensor,
                                 sensor::Sensor* distSensor)
: mLatitude(latitude), mLongitude(longitude)
, mLatitudeSensor(latitudeSensor), mLongitudeSensor(longitudeSensor), mDistSensor(distSensor)
, RX_BUF_LEN(std::max(InitialMsg::FRAME_SIZE, FinalMsg::FRAME_SIZE)) {
    assert(RX_BUF_LEN <= getRxBufferSize());
    rx_buffer = getRxBuffer();
}
UwbAnchorDevice::~UwbAnchorDevice() {
}

void UwbAnchorDevice::setup() {
    Dw3000Device::setup();

    setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
}

void UwbAnchorDevice::loop() {
    Dw3000Device::loop();

    if (getDiagnosticStatus() == DIAG_OK) {
        do_ranging();

        if (!mHighFreqLoopRequester.is_high_frequency()) {
            // do background work only when not running at high frequency (due to active ranging)
            maybe_reportPosition();
        }
    }
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
            case MYSTATE_SEND_ERROR_RESPONSE:             newStateStr = "SEND_ERROR_RESPONSE"; logLvl = ESPHOME_LOG_LEVEL_WARN; break;
            case MYSTATE_WAIT_RECV_FINAL:                 newStateStr = "WAIT_RECV_FINAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_FRAME_FINAL:               newStateStr = "RECVD_FRAME_FINAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_FRAME_VALID_FINAL:         newStateStr = "RECVD_FRAME_VALID_FINAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_RECVD_FRAME_INVALID_FINAL:       newStateStr = "RECVD_FRAME_INVALID_FINAL"; logLvl = ESPHOME_LOG_LEVEL_WARN; break;
            case MYSTATE_SENT_FINAL:                      newStateStr = "SENT_FINAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_SEND_ERROR_FINAL:                newStateStr = "SEND_ERROR_FINAL"; logLvl = ESPHOME_LOG_LEVEL_WARN; break;
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

void UwbAnchorDevice::maybe_reportPosition() {
    const uint32_t uptime = millis();
    if (   (mLastPositionReportedMs == 0) // never reported so far
        || (uptime - mLastPositionReportedMs) >= 30000U) { // every 30s
        mLastPositionReportedMs = uptime;

        if (mLatitudeSensor != nullptr) {
            mLatitudeSensor->publish_state(mLatitude);
        }
        if (mLongitudeSensor != nullptr) {
            mLongitudeSensor->publish_state(mLongitude);
        }
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
        case MYSTATE_SEND_ERROR_RESPONSE:          sendErrorResponse(); break;
        case MYSTATE_WAIT_RECV_FINAL:              waitRecvFinal(); break;
        case MYSTATE_RECVD_FRAME_FINAL:            recvdFrameFinal(); break;
        case MYSTATE_RECVD_FRAME_VALID_FINAL:      recvdFrameValidFinal(); break; // should not occur because direct call
        case MYSTATE_RECVD_FRAME_INVALID_FINAL:    recvdFrameInvalidFinal(); break;
        case MYSTATE_SENT_FINAL:                   sentFinal(); break;
        case MYSTATE_SEND_ERROR_FINAL:             sendErrorFinal(); break;
        default:
            ESP_LOGE(TAG, "unhandled state %d", currState);
            // reset state machine
            setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
            break;
    }
}

void UwbAnchorDevice::prepareWaitRecvInitial() {
    mHighFreqLoopRequester.stop();

    /* Activate reception immediately. */
    dwt_setpreambledetecttimeout(0);
    dwt_setrxtimeout(0);
    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) == DWT_ERROR) {
        const uint32_t status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        ESP_LOGE(TAG, "dwt_rxenable failed: SYS_STATUS: 0x%08lx", status_reg);
    } else {
        setMyState(MYSTATE_WAIT_RECV_INITIAL);
    }
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

        mLastInitialReceivedMicros = micros();

        // immediate call instead of state change
        recvdFrameInitial();
    }  else {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        const uint32_t waitMicros = micros() - startedPollLoopMicros;
        const std::string & rxErrors = getRxErrorString(status_reg);
        ESP_LOGW(TAG, "waitRecvInitial RX %s after %" PRIu32 " us", rxErrors.c_str(), waitMicros);
        if (status_reg & SYS_STATUS_ALL_RX_TO) {
            // don't change state on RX timeout
        } else {
            setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
        }
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
    bool proceed = mInitialFrame.fromIncomingBytes(rx_buffer, frame_len);
    if (proceed) {
        proceed = mInitialFrame.isValid();
    }
    if (proceed) {
        if (mInitialFrame.getTargetId() != getDeviceId()) {
            // silently ignore
            proceed = false;
            setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
            return;
        } else {
            // remember the source ID as the tag's device ID
            mCurrentTagId = mInitialFrame.getSourceId();
        }
    }
    if (proceed) {
        /* Yes, it is the frame we are expecting. */
        setMyState(MYSTATE_RECVD_FRAME_VALID_INITIAL);

        /* Retrieve Initial reception timestamp. */
        mInitial_rx_ts = get_rx_timestamp_u64();

        uint8_t fctCode;
        uint16_t fctData;
        mInitialFrame.getFunctionCodeAndData(fctCode, fctData);
        switch (fctCode) {
            case InitialMsg::INITIAL_FCT_CODE_ANT_DELAY_CALIBRATION:
                if (getMode() != UWB_MODE_ANT_DELAY_CALIBRATION) {
                    setMode(UWB_MODE_ANT_DELAY_CALIBRATION);
                }
                setCalibrationAntennaDelays(fctData);
                break;
            case InitialMsg::INITIAL_FCT_CODE_RANGING:
            default:
                if (getMode() != UWB_MODE_RANGING) {
                    setMode(UWB_MODE_RANGING); // also resets antenna delays back to default
                }
                break;
        }

        TIME_CRITICAL_START();

        /* Compute Response message delayed transmission time. */
        uint32_t INITIAL_RX_TO_RESP_TX_DLY_UUS_TEMP = INITIAL_RX_TO_RESP_TX_DLY_UUS;
        const uint64_t response_tx_time =
            ((mInitial_rx_ts + ((uint64_t)INITIAL_RX_TO_RESP_TX_DLY_UUS_TEMP * UUS_TO_DWT_TIME)) & 0x00FFFFFFFFFFFFFFUL) >> 8;
        dwt_setdelayedtrxtime((uint32_t)response_tx_time);

        /* Set expected delay and timeout for Final message reception. */
        dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
        dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
        /* Set preamble timeout for expected frames. */
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);

        /* Write and send the response message. */
        mResponseFrame.resetToDefault();
        mResponseFrame.setSequenceNumber(Dw3000Device::getNextTxSequenceNumberAndIncrease());
        mResponseFrame.setTargetId(mCurrentTagId);
        mResponseFrame.setSourceId(getDeviceId());
        /* Set function code and data (max 2 bytes)
           -- reserved for future use --
        */
        // not checking mResponseFrame.isValid() in order to save processing time
        uint8_t* txbuffer = mResponseFrame.getBytes().data();
        dwt_writetxdata(ResponseMsg::FRAME_SIZE, txbuffer, 0 /*zero offset*/);
        dwt_writetxfctrl(ResponseMsg::FRAME_SIZE, 0 /*zero offset*/, 1 /* 1=ranging */);

        const uint32_t systime = (uint64_t) dwt_readsystimestamphi32();
        /* Clear any TX events */
        dwt_write8bitoffsetreg(SYS_STATUS_ID, 0, (uint8_t)SYS_STATUS_ALL_TX);
        bool hpdWarning = false, txError = false;
        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
        const int rc = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED, &hpdWarning, &txError);
        const uint64_t rangingDurationSoFarUs = micros() - mLastInitialReceivedMicros;
        if (DWT_ERROR == rc) {
            mTxErrorCount++;
            ESP_LOGE(TAG, "Response TX_DELAYED failed (total %" PRIu32 "x)  HDPWARN:%d TXERR:%d", mTxErrorCount, hpdWarning, txError);
            const int32_t diff = (uint32_t)response_tx_time - systime; // diff should be positive in good case
            ESP_LOGE(TAG, "systime=%" PRIu32 ", response_tx_time=%" PRIu64 ", diff=%" PRId32
                " INITIAL_RX_TO_RESP_TX_DLY_UUS:%" PRIu32 " after %" PRIu64 "us",
                systime, response_tx_time, diff, INITIAL_RX_TO_RESP_TX_DLY_UUS_TEMP, rangingDurationSoFarUs);
            setMyState(MYSTATE_SEND_ERROR_RESPONSE);
        } else {
            TIME_CRITICAL_END();
            const int32_t diff = (uint32_t)response_tx_time - systime; // diff should be positive in good case
            ESP_LOGV(TAG, "Response sent OK: systime=%" PRIu32 ", final_tx_time=%" PRIu64 ", diff=%" PRId32 " INITIAL_RX_TO_RESP_TX_DLY_UUS:%" PRIu32,
                systime, response_tx_time, diff, INITIAL_RX_TO_RESP_TX_DLY_UUS_TEMP);
            setMyState(MYSTATE_SENT_RESPONSE);

            mEnteredWaitRecvFinalMicros = micros();
            setMyState(MYSTATE_WAIT_RECV_FINAL);
#if ANCHOR_USE_IMMEDIATE_CALLS == 1
            // immediate call instead of state change
            waitRecvFinal();
#endif // ANCHOR_USE_IMMEDIATE_CALLS
        }
    } else {
        setMyState(MYSTATE_RECVD_FRAME_INVALID_INITIAL);
    }
}

void UwbAnchorDevice::waitRecvFinal() {
    /* Check for timeout */
    const uint32_t startedPollLoopMicros = micros();
    if (startedPollLoopMicros - mEnteredWaitRecvFinalMicros >= (WAIT_FINAL_RX_TIMEOUT_MS * 1000U)) {
        /* Abort this ranging attempt */
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
        setMyState(MYSTATE_RECVD_FRAME_FINAL);

        /* Clear good RX frame event in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        // immediate call instead of state change
        recvdFrameFinal();
    }  else {
        /* Clear RX error/timeout events in the DW IC status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

        const uint32_t waitMicros = micros() - mEnteredWaitRecvFinalMicros;
        const std::string & rxErrors = getRxErrorString(status_reg);
        ESP_LOGE(TAG, "waitRecvFinal RX %s after %" PRIu32 " us", rxErrors.c_str(), waitMicros);
        // abort this ranging attempt
        setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
    }
}

void UwbAnchorDevice::recvdFrameFinal() {
    /* A frame has been received, read it into the local buffer. */
    const uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    } else {
        ESP_LOGW(TAG, "recvd frame bytes %" PRIu16 " exceeds RX_BUF_LEN %zu", frame_len, RX_BUF_LEN);
        setMyState(MYSTATE_RECVD_FRAME_INVALID_INITIAL);
        return;
    }
    /* Check that the frame is a Final frame sent by inititating tag device and targeted to this device. */
    bool proceed = mFinalFrame.fromIncomingBytes(rx_buffer, frame_len);
    if (proceed) {
        proceed = mFinalFrame.isValid();
    }
    const uint8_t thisDeviceId = getDeviceId();
    const uint8_t otherDeviceId = mFinalFrame.getSourceId();
    if (proceed) {
        if ( (mFinalFrame.getTargetId() != thisDeviceId) || (otherDeviceId != mCurrentTagId) ) {
            // silently ignore
            proceed = false;
            setMyState(MYSTATE_WAIT_RECV_FINAL); // keep state, not MYSTATE_RECVD_FRAME_INVALID_FINAL
            return;
        }
    }
    if (proceed) {
        /* Yes, it is the frame we are expecting. */
        setMyState(MYSTATE_RECVD_FRAME_VALID_FINAL);

        TIME_CRITICAL_START();

        /* Retrieve Response transmission and Final reception timestamps. */
        const uint64_t response_tx_ts = get_tx_timestamp_u64();
        const uint64_t final_rx_ts = get_rx_timestamp_u64();

        /* Get timestamps embedded in the final message. */
        uint64_t initial_tx_time, resp_rx_time, final_tx_time;
        mFinalFrame.getTimestamps(&initial_tx_time, &resp_rx_time, &final_tx_time);

        /* Compute Final response message delayed transmission time. */
        const uint32_t FINAL_RX_TO_FINAL_TX_DLY_UUS_TEMP = FINAL_RX_TO_FINAL_TX_DLY_UUS; //1700 + ((mRangingLoopCnt % 80) * 10);
        const uint64_t final_response_tx_time =
            ((response_tx_ts + ((uint64_t)FINAL_RX_TO_FINAL_TX_DLY_UUS_TEMP * UUS_TO_DWT_TIME))  & 0x00FFFFFFFFFFFFFFUL) >> 8;
        dwt_setdelayedtrxtime((uint32_t)final_response_tx_time);

        /* Final response TX timestamp is the transmission time we programmed plus the TX antenna delay. */
        const uint64_t final_response_tx_ts = (((uint64_t)(final_response_tx_time & 0xFFFFFFFEUL)) << 8) + getAntennaDelay();

        /* Write all timestamps in the Final response message. */
        mFinalFrame.resetToDefault();
        mFinalFrame.setSequenceNumber(Dw3000Device::getNextTxSequenceNumberAndIncrease());
        mFinalFrame.setTimestamps(response_tx_ts, final_rx_ts, final_response_tx_ts);
        mFinalFrame.setTargetId(otherDeviceId);
        mFinalFrame.setSourceId(thisDeviceId);

        /* Compute time of flight. */
        const double Ra = (double)(resp_rx_time - initial_tx_time);
        const double Rb = (double)(final_rx_ts - response_tx_ts);
        const double Da = (double)(final_tx_time - resp_rx_time);
        const double Db = (double)(response_tx_ts - mInitial_rx_ts);
        const int64_t tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
        const double tof = tof_dtu * DWT_TIME_UNITS;
        double distance = tof * SPEED_OF_LIGHT;
        if (Location::isDistancePlausible(distance) || getMode() == UWB_MODE_ANT_DELAY_CALIBRATION) {
            // write distance in [cm] to Final frame
            const int16_t dist_cm = (int16_t)(distance * 100.0); // [m] -> [cm]
            const uint8_t fctData[FinalMsg::FINAL_DATA_SIZE] = {
                (uint8_t)((dist_cm & 0xFF00U) >> 8),
                (uint8_t)((dist_cm & 0x00FFU))
            };
            mFinalFrame.setFunctionCodeAndData(FinalMsg::FINAL_FCT_CODE_RANGING_DIST, fctData, FinalMsg::FINAL_DATA_SIZE);
        }

        /* Write and send Final response message. */
        // not checking mFinalFrame.isValid() in order to save processing time
        uint8_t* txbuffer = mFinalFrame.getBytes().data();
        dwt_writetxdata(FinalMsg::FRAME_SIZE, txbuffer, 0 /*zero offset*/);
        dwt_writetxfctrl(FinalMsg::FRAME_SIZE, 0 /*zero offset*/, 1 /*=ranging */);
        /* Clear any TX events */
        dwt_write8bitoffsetreg(SYS_STATUS_ID, 0, (uint8_t)SYS_STATUS_ALL_TX);
        const uint32_t systime = dwt_readsystimestamphi32();
        bool hpdWarning = false, txError = false;
        const int rc = dwt_starttx(DWT_START_TX_DELAYED, &hpdWarning, &txError);
        const uint64_t rangingDurationUs = micros() - mLastInitialReceivedMicros;
        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
        if (DWT_SUCCESS == rc) {
            TIME_CRITICAL_END();
            setMyState(MYSTATE_SENT_FINAL);
            ESP_LOGV(TAG, "Final response sent Ok: systime=%" PRIu32 ", final_response_tx_time=%" PRIu64 ", diff=%" PRId32 " FINAL_RX_TO_FINAL_TX_DLY_UUS:%" PRIu32 ,
                    systime, final_response_tx_time, ((int32_t)(final_response_tx_time-systime)), FINAL_RX_TO_FINAL_TX_DLY_UUS_TEMP);
        } else {
            TIME_CRITICAL_END();
            mTxErrorCount++;
            ESP_LOGE(TAG, "Final response TX_DELAYED failed (total %" PRIu32 "x) HDPWARN:%d TXERR:%d", mTxErrorCount, hpdWarning, txError);
            const int32_t diff = (uint32_t)final_response_tx_time - systime; // diff should be positive in good case
            const uint64_t final_rx_time = (final_rx_ts & 0x00FFFFFFFFFFFFFFUL) >> 8;
            ESP_LOGE(TAG, "systime=%" PRIu32 ", final_response_tx_time=%" PRIu64 ", diff=%" PRId32
                " FINAL_RX_TO_FINAL_TX_DLY_UUS:%" PRIu32 " after %" PRIu64 " us",
                systime, final_response_tx_time, diff, FINAL_RX_TO_FINAL_TX_DLY_UUS_TEMP, rangingDurationUs);
            ESP_LOGW(TAG, "final_rx_time=%" PRIu64 ", diff=%" PRId32, final_rx_time, ((uint32_t)final_rx_time - systime));
            setMyState(MYSTATE_SEND_ERROR_FINAL);
        }

        /* Plausibility check. */
        if (Location::isDistancePlausible(distance) || getMode() == UWB_MODE_ANT_DELAY_CALIBRATION) {
            /* Display computed distance. */
            ESP_LOGW(TAG, "DIST tag 0x%02X: %.2fm took %" PRIu64 " us", otherDeviceId, distance, rangingDurationUs);
        } else {
            ESP_LOGW(TAG, "DIST tag 0x%02X: %.2fm implausible (>%.0f) took %" PRIu64 " us", otherDeviceId, distance,
                Location::UWB_MAX_REACH_METER, rangingDurationUs);
            distance = NAN;
        }
        /* Report distance sensor. */
        if (mDistSensor != nullptr) {
            (const_cast<sensor::Sensor*>(mDistSensor))->publish_state(distance);
        }
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

void UwbAnchorDevice::sendErrorResponse() {
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

void UwbAnchorDevice::sentFinal() {
    /* Next ranging cycle. */
    setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
}

void UwbAnchorDevice::sendErrorFinal() {
    // reset state machine
    setMyState(MYSTATE_PREPARE_WAIT_RECV_INITIAL);
}

}  // namespace uwb
}  // namespace esphome

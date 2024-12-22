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

#include "UwbTagDevice.h"

#include <iomanip>
#include <sstream>

#include "InitialMsg.h"
#include "ResponseMsg.h"
#include "FinalMsg.h"

#include "esphome/core/log.h"

#include "dw3000.h"

/* time between ranging attempts with next anchor. */
#define INTER_ANCHOR_RANGING_INTERVAL   (100U)
/* per anchor: count of retries. */
#define PER_ANCHOR_ATTEMPTS             (3U)

namespace esphome {
namespace uwb {

const char* UwbTagDevice::TAG = "tag";
const char* UwbTagDevice::STATE_TAG = "tag_STATE";

UwbTagDevice::UwbTagDevice(const std::vector<std::shared_ptr<UwbAnchorData>> & anchors,
                           const uint32_t rangingIntervalMs,
                           const uint32_t maxAgeAnchorDistanceMs,
                           sensor::Sensor* latitudeSensor,
                           sensor::Sensor* longitudeSensor,
                           sensor::Sensor* locationErrorEstimateSensor,
                           sensor::Sensor* anchorsInUseSensor,
                           AntDelayCalibDistanceNumber* antennaCalibrationDistanceNumber,
                           AntDelayCalibDeviceSelect* antennaCalibrationDeviceSelect,
                           AntDelayCalibStartButton* antennaCalibrationStartButton,
                           sensor::Sensor* antennaCalibrationProgress,
                           sensor::Sensor* antennaCalibrationDelayResultSensor)
: RX_BUF_LEN(std::max(ResponseMsg::FRAME_SIZE, FinalMsg::FRAME_SIZE)),
  RANGING_INTERVAL_MS(rangingIntervalMs),
  MAX_AGE_ANCHOR_DISTANCE_MS(maxAgeAnchorDistanceMs)
{
    mAnchors = std::move(anchors);
    mAnchorCurrentRangingSuccess.reserve(mAnchors.size());
    mAnchorCurrentRangingSuccess.assign(mAnchors.size(), PER_ANCHOR_ATTEMPTS);
    assert(RX_BUF_LEN <= getRxBufferSize());
    rx_buffer = getRxBuffer();
    mLatitudeSensor = latitudeSensor;
    mLongitudeSensor = longitudeSensor;
    mLocationErrorEstimateSensor = locationErrorEstimateSensor;
    mAnchorsInUseSensor = anchorsInUseSensor;
    mAntDelayCalibrationResultPerRound.reserve(ANT_CALIB_MAX_ROUNDS);
    mAntDelayCalibrationResultPerRound.clear();
    mAntennaCalibrationDistanceNumber = antennaCalibrationDistanceNumber;
    mAntDelayCalibDeviceSelect = antennaCalibrationDeviceSelect;
    mAntDelayStartButton = antennaCalibrationStartButton;
    mAntennaCalibrationProgressSensor = antennaCalibrationProgress;
    mAntennaCalibrationDelayResultSensor = antennaCalibrationDelayResultSensor;
}

UwbTagDevice::~UwbTagDevice() {
}

void UwbTagDevice::setup() {
    Dw3000Device::setup();

    setMyState(MY_DEFAULT_STATE);

    if (mAntennaCalibrationDistanceNumber != nullptr) {
        mAntennaCalibrationDistanceNumber->publish_state(mAntDelayCalibration.getCalibrationDistance());
    }

    if (mAntDelayCalibDeviceSelect != nullptr) {
        std::vector<std::string> options;
        for (auto & anchor : mAnchors) {
            const uint8_t id = anchor->getId();
            std::ostringstream option;
            option << std::uppercase << std::hex << +id << std::dec;
            options.push_back(option.str());
        }
        mAntDelayCalibDeviceSelect->traits.set_options(options);
    }
}

void UwbTagDevice::loop() {
    Dw3000Device::loop();

    if (getDiagnosticStatus() == DIAG_OK) {
        if (getMode() != UWB_MODE_ANT_DELAY_CALIBRATION_DONE) {
            do_ranging();
        }
    }
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
            case MYSTATE_WAIT_NEXT_RANGING_INTERVAL:newStateStr = "WAIT_NEXT_RANGING_INTERVAL"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_WAIT_NEXT_ANCHOR_RANGING:  newStateStr = "WAIT_NEXT_ANCHOR_RANGING"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
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
            case MYSTATE_CALCULATE_LOCATION_PREPARE:newStateStr = "CALCULATE_LOCATION_PREPARE"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_CALCULATE_LOCATION_PHASES: newStateStr = "CALCULATE_LOCATION_PHASES"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
            case MYSTATE_CALCULATE_LOCATION_POST:   newStateStr = "CALCULATE_LOCATION_POST"; logLvl = ESPHOME_LOG_LEVEL_INFO; break;
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
        case MYSTATE_WAIT_NEXT_RANGING_INTERVAL: waitNextRangingInterval(); break;
        case MYSTATE_WAIT_NEXT_ANCHOR_RANGING:   waitNextAnchorRanging(); break;
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
        case MYSTATE_CALCULATE_LOCATION_PREPARE: calculateLocationPrepare(); break;
        case MYSTATE_CALCULATE_LOCATION_PHASES:  calculateLocationInPhases(); break;
        case MYSTATE_CALCULATE_LOCATION_POST:    locationPostProcessing(); break;
        default:
            ESP_LOGE(TAG, "unhandled state %d", currState);
            // reset state machine
            setMyState(MY_DEFAULT_STATE);
            break;
    }
}

uint32_t UwbTagDevice::getRangingInterval() const {
    if (getMode() == UWB_MODE_ANT_DELAY_CALIBRATION) {
        return INTER_ANCHOR_RANGING_INTERVAL;
    } else {
        return RANGING_INTERVAL_MS;
    }
}

void UwbTagDevice::waitNextRangingInterval() {
    const uint32_t now = millis();
    if ((now - mLastRangingIntervalStartedMillis) >= getRangingInterval()) {
        mLastRangingIntervalStartedMillis = now;
        mCurrentAnchorIndex = 0;
        mAnchorCurrentRangingSuccess.assign(mAnchors.size(), PER_ANCHOR_ATTEMPTS);
        setMyState(MYSTATE_WAIT_NEXT_ANCHOR_RANGING);
    }
    // else continue waiting

    if (getMode() == UWB_MODE_ANT_DELAY_CALIBRATION) {
        if (mCurrentAntCalibAnchorIndex >= 0 && mCurrentAntCalibAnchorIndex < mAnchors.size() && mAntDelayCalibDeviceSelect != nullptr) {
            const uint8_t id = mAnchors.at(mCurrentAntCalibAnchorIndex)->getId();
            std::ostringstream currentDevice;
            currentDevice << std::uppercase << std::hex << +id << std::dec;
            mAntDelayCalibDeviceSelect->publish_state(currentDevice.str());
        }
        if ((!mAntDelayCalibration.isDone()) && mAntennaCalibrationDelayResultSensor != nullptr) {
            mAntennaCalibrationDelayResultSensor->publish_state(NAN);
        }
    }
}

void UwbTagDevice::waitNextAnchorRanging() {
    const uint32_t now = millis();
    if ((now - mLastInitialSentMillis) >= INTER_ANCHOR_RANGING_INTERVAL) {
        switch(getMode()) {

            case UWB_MODE_ANT_DELAY_CALIBRATION:
            {
                if (mAntDelayCalibration.isDone()) {
                    // store store result of this round
                    const uint16_t adelay = mAntDelayCalibration.getAntennaDelay();
                    mAntDelayCalibrationResultPerRound.push_back((double)adelay);

                    if (mAntDelayCalibrationResultPerRound.size() < ANT_CALIB_MAX_ROUNDS) {
                        // prepare next round
                        mAntDelayCalibration.resetState();
                    } else {
                        // all rounds finished
                        double meanCalibrationResult;
                        const double stdDevCalibrationResults =
                            AntDelayCalibration::getStandardDeviationAndMeanValue(mAntDelayCalibrationResultPerRound, meanCalibrationResult);
                        const uint16_t calibrationResult = (uint16_t) std::round(meanCalibrationResult);
                        // report result
                        std::ostringstream msg;
                        msg << +calibrationResult << " (StdDev: "
                            << std::fixed << std::setprecision(2) << +stdDevCalibrationResults
                            << ") @ dist " << +mAntDelayCalibration.getCalibrationDistance() << "m";
                        ESP_LOGW(TAG, ">>>>>> ANTENNA DELAY CALIBRATION: %s", msg.str().c_str());
                        sendLog(msg.str());

                        // reset my variables
                        mAntDelayCalibration.resetState();
                        mAntDelayCalibrationResultPerRound.clear();

                        // Done
                        setMyState(MY_DEFAULT_STATE);
                        setMode(UWB_MODE_ANT_DELAY_CALIBRATION_DONE);
                        if (mAntennaCalibrationProgressSensor != nullptr) {
                            mAntennaCalibrationProgressSensor->publish_state(100.0);
                        }
                        if (mAntennaCalibrationDelayResultSensor != nullptr) {
                            mAntennaCalibrationDelayResultSensor->publish_state((float)calibrationResult);
                        }
                        return;
                    }
                }
                // always choose the same anchor for antenna delay calibration
                if ((mCurrentAntCalibAnchorIndex >=0) && (mCurrentAntCalibAnchorIndex < mAnchors.size())) {
                    mCurrentAnchorIndex = mCurrentAntCalibAnchorIndex;
                    setMyState(MYSTATE_PREPARE_SEND_INITIAL);
                    // don't wait until next loop() but start immediately
                    prepareSendInitial();
                } else {
                    ESP_LOGE(TAG, "mode ANT_DELAY_CALIBRATION but anchor index %u", mCurrentAntCalibAnchorIndex);
                    // w.t.f. get out of here, but don't flood the log
                    delay(500);
                    return;
                }
            }
            break;

            case UWB_MODE_RANGING:
            {
                // find next anchor to do ranging with. There is at least 1 anchor, otherwise we would not be here
                unsigned nextAnchorIndex;
                for (nextAnchorIndex = 0; nextAnchorIndex < mAnchors.size(); nextAnchorIndex++) {
                    if (mAnchorCurrentRangingSuccess[nextAnchorIndex] > 0) {
                        break; // found next one
                    }
                }
                if (nextAnchorIndex >= mAnchors.size()) {
                    // ranging done with all anchors, calculate resulting position
                    setMyState(MYSTATE_CALCULATE_LOCATION_PREPARE);
                } else {
                    mCurrentAnchorIndex = nextAnchorIndex;
                    setMyState(MYSTATE_PREPARE_SEND_INITIAL);
                    // don't wait until next loop() but start immediately
                    prepareSendInitial();
                }
            }
            break;

            case UWB_MODE_ANT_DELAY_CALIBRATION_DONE:
            default:
            {
                // should not happen
                setMyState(MY_DEFAULT_STATE);
                return;
            }
        } // switch getMode()
    }
    // else continue waiting
}

void UwbTagDevice::prepareSendInitial() {
    if (mCurrentAnchorIndex < 0 || mCurrentAnchorIndex >= mAnchors.size()) {
        ESP_LOGE(TAG, "Tag 0x%02X: mCurrentAnchorIndex out of range: %i", getDeviceId(), mCurrentAnchorIndex);
        return;
    }
    const uint32_t now = millis();
    const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
    ESP_LOGI(TAG, "Tag 0x%02X: Initiating to Anchor 0x%02X", getDeviceId(), anchorId);

    TIME_CRITICAL_START();
    /* Set expected Response's delay and timeout. */
    dwt_setrxaftertxdelay(INITIAL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    dwt_setpreambledetecttimeout(PRE_TIMEOUT);

    /* Prepare Initial frame. */
    mInitialFrame.resetToDefault();
    mInitialFrame.setSequenceNumber(getNextTxSequenceNumberAndIncrease());
    mInitialFrame.setTargetId(anchorId);
    mInitialFrame.setSourceId(getDeviceId());
    switch (getMode()) {
        case UWB_MODE_ANT_DELAY_CALIBRATION:
        {
            if (mAntDelayCalibration.isDone()) {
                // new calibration round
                mAntDelayCalibration.resetState();
            }
            const uint16_t nextCalibAntennaDelay = mAntDelayCalibration.getNextAntennaDelay();
            mInitialFrame.setFunctionCodeAndData(InitialMsg::INITIAL_FCT_CODE_ANT_DELAY_CALIBRATION, nextCalibAntennaDelay);
            setCalibrationAntennaDelays(nextCalibAntennaDelay);
        }
        break;
        case UWB_MODE_RANGING:
        {
            mInitialFrame.setFunctionCodeAndData(InitialMsg::INITIAL_FCT_CODE_RANGING, 0);
        }
        break;
        case UWB_MODE_ANT_DELAY_CALIBRATION_DONE:
        default:
        {
            // should not happen
            setMyState(MY_DEFAULT_STATE);
            return;
        }
    } // switch
    if (mInitialFrame.isValid()) {
        uint8_t* tx_buffer = mInitialFrame.getBytes().data();
        /* Clear Transmit Frame Sent. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        /* Write frame data to DW IC and prepare transmission. */
        if (dwt_writetxdata(InitialMsg::FRAME_SIZE, tx_buffer, 0 /*zero offset*/) != DWT_SUCCESS) {
            TIME_CRITICAL_END();
            mTxErrorCount++;
            ESP_LOGE(TAG, "Write TX failed (total TX errors %" PRIu32 "x)", mTxErrorCount);
            setMyState(MYSTATE_SEND_ERROR_INITIAL);
            return;
        }
        dwt_writetxfctrl(InitialMsg::FRAME_SIZE, 0 /*zero offset*/, 1 /*=ranging*/);
        /* Start transmission, indicating that a response is expected so that reception is enabled automatically
           after the frame is sent and the delay set by dwt_setrxaftertxdelay() has elapsed. */
        if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) == DWT_SUCCESS) {
            TIME_CRITICAL_END();
            mLastInitialSentMillis = now; // set time of when this function was entered, ensures a more acurrate INTER_ANCHOR_RANGING_INTERVAL
            mHighFreqLoopRequester.start(); // must be able to receive Response within microseconds
            setMyState(MYSTATE_SENT_INITIAL);
        } else {
            TIME_CRITICAL_END();
            mTxErrorCount++;
            ESP_LOGE(TAG, "TX_IMMEDIATE failed (total TX errors %" PRIu32 "x)", mTxErrorCount);
            setMyState(MYSTATE_SEND_ERROR_INITIAL);
        }
    } else {
        TIME_CRITICAL_END();
        ESP_LOGE(TAG, "Initial frame invalid");
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

void UwbTagDevice::waitRecvResponse() {
    /* Check for timeout */
    const uint32_t startedPollLoopMicros = micros();
    const uint32_t diffToEnterStateMicros = startedPollLoopMicros - mEnteredWaitRecvResponseMicros;
    if (diffToEnterStateMicros >= (WAIT_RX_TIMEOUT_MS * 1000U)) {
        // next ranging loop
        ESP_LOGI(TAG, "Timeout awaiting Response after %" PRIu32 " ms", (diffToEnterStateMicros/1000U));
        rangingDone(false);
        return;
    }
    /* Poll for reception of expected Response frame or error/timeout. */
    uint32_t status_reg;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        if ((micros() - startedPollLoopMicros) >= (MAX_POLL_DURATION_MS * 1000U)) {
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
                ESP_LOGE(TAG, "0x%02X: waitRecvResponse RX Frame Wait timeout after %" PRIu32 " us", anchorId, waitMicros);
            if ((status_reg & SYS_STATUS_RXPTO_BIT_MASK) == SYS_STATUS_RXPTO_BIT_MASK)
                ESP_LOGE(TAG, "0x%02X: waitRecvResponse RX Preamble Detection timeout after %" PRIu32 " us", anchorId, waitMicros);
        } else if (status_reg & SYS_STATUS_ALL_RX_ERR) {
            ESP_LOGE(TAG, "0x%02X: waitRecvResponse RX error after %" PRIu32 " us", anchorId, waitMicros);
        } else {
            ESP_LOGE(TAG, "0x%02X: waitRecvResponse status_reg=0x%08x after %" PRIu32 " us", anchorId, status_reg, waitMicros);
        }
        rangingDone(false);
    }
}

void UwbTagDevice::recvdFrameResponse() {
    /* A frame has been received, read it into the local buffer. */
    const uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    } else {
        const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
        ESP_LOGE(TAG, "0x%02X: recvdFrameResponse frame bytes %" PRIu16 " exceeds RX_BUF_LEN %zu", anchorId, frame_len, RX_BUF_LEN);
        rangingDone(false);
        return;
    }
    /* Check that the frame is the expected Response and targeted to this device. */
    bool proceed = mResponseFrame.fromIncomingBytes(rx_buffer, frame_len);
    if (proceed) {
        proceed = mResponseFrame.isValid();
    }
    uint8_t anchorId;
    if (proceed) {
        if (mResponseFrame.getTargetId() != getDeviceId()) {
            proceed = false;
        }
        anchorId = mResponseFrame.getSourceId();
        // skipping the check for anchorId against current mCurrentAnchorIndex to save processing time
    }
    if (proceed) {
        /* Yes, it is the frame we are expecting. */
        setMyState(MYSTATE_RECVD_VALID_RESPONSE);

        TIME_CRITICAL_START();
        /* Retrieve reception timestamp of this incoming frame. */
        const uint64_t response_rx_ts = get_rx_timestamp_u64();

        /* Compute Final message delayed transmission time. */
        const uint64_t final_tx_time =
            ((response_rx_ts + ((uint64_t)RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) & 0x00FFFFFFFFFFFFFFUL) >> 8;
        dwt_setdelayedtrxtime((uint32_t)final_tx_time);

        /* Set expected Final's delay and timeout. */
        dwt_setrxaftertxdelay(FINAL_TX_TO_FINAL_RESPONSE_RX_DLY_UUS);
        dwt_setrxtimeout(FINAL_RESPONSE_RX_TIMEOUT_UUS);
        dwt_setpreambledetecttimeout(PRE_TIMEOUT);

        /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
        const uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + getAntennaDelay();

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
        mResponse_rx_ts = response_rx_ts;
        if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) == DWT_SUCCESS) {
            TIME_CRITICAL_END();
            setMyState(MYSTATE_SENT_FINAL);
            ESP_LOGV(TAG, "Final sent Ok: systime=%" PRIu32 ", final_tx_time=%" PRIu64 ", diff=%" PRId32 ,
                    systime, final_tx_time, (int32_t)(final_tx_time-systime));
        } else {
            TIME_CRITICAL_END();
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

        // -- reserved for future use --
        /* after(!) trying to send Final frame, read function code and data from Response frame */
        uint8_t data[ResponseMsg::RESPONSE_DATA_SIZE];
        std::size_t actualdataSize;
        uint8_t fctCode;
        if (mResponseFrame.getFunctionCodeAndData(&fctCode, data, ResponseMsg::RESPONSE_DATA_SIZE, &actualdataSize)) {
            if (fctCode == ResponseMsg::RESPONSE_FCT_CODE_RANGING) {
                const uint16_t dummyData = (uint16_t)(data[0] << 8) + (uint16_t) data[1];
            } else {
                ESP_LOGE(TAG, "0x%02X: getFunctionCodeAndData from Response frame failed", anchorId);
            }
        } else {
            ESP_LOGE(TAG, "0x%02X: getFunctionCodeAndData from Response frame failed", anchorId);
        }
    } else {
        const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
        ESP_LOGW(TAG, "0x%02X: recvdFrameResponse invalid", anchorId);
        setMyState(MYSTATE_RECVD_INVALID_RESPONSE);
    }
}

void UwbTagDevice::sendInitialError() {
    rangingDone(false);
}

void UwbTagDevice::waitRecvFinal() {
    /* Check for timeout */
    const uint32_t startedPollLoopMicros = micros();
    const uint32_t diffToEnterStateMicros = startedPollLoopMicros - mEnteredWaitRecvFinalMicros;
    if (diffToEnterStateMicros >= (WAIT_RX_TIMEOUT_MS * 1000U)) {
        ESP_LOGW(TAG, "Timeout awaiting Final after %" PRIu32 " ms", (diffToEnterStateMicros/1000U));
        rangingDone(false);
        return;
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
                ESP_LOGE(TAG, "0x%02X: waitRecvFinal RX Frame Wait timeout after %" PRIu32 " us", anchorId, waitMicros);
            if ((status_reg & SYS_STATUS_RXPTO_BIT_MASK) == SYS_STATUS_RXPTO_BIT_MASK)
                ESP_LOGE(TAG, "0x%02X: waitRecvFinal RX Preamble Detection timeout after %" PRIu32 " us", anchorId, waitMicros);
        } else if (status_reg & SYS_STATUS_ALL_RX_ERR) {
            ESP_LOGE(TAG, "0x%02X: waitRecvFinal RX error after %" PRIu32 " us", anchorId, waitMicros);

        } else {
            ESP_LOGE(TAG, "0x%02X: waitRecvFinal status_reg=0x%08x after %" PRIu32 " us", anchorId, status_reg, waitMicros);
        }
        rangingDone(false);
    }
}

void UwbTagDevice::recvdFrameFinal() {
    /* A frame has been received, read it into the local buffer. */
    const uint16_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN) {
        dwt_readrxdata(rx_buffer, frame_len, 0);
    } else {
        const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
        ESP_LOGE(TAG, "0x%02X: recvdFrameFinal frame bytes %" PRIu16 " exceeds RX_BUF_LEN %zu", anchorId, frame_len, RX_BUF_LEN);
        rangingDone(false);
        return;
    }
    /* Check that the frame is the expected Final response and targeted to this device. */
    bool proceed = mFinalFrame.fromIncomingBytes(rx_buffer, frame_len);
    if (proceed) {
        bool proceed = mFinalFrame.isValid();
    }
    uint8_t anchorId;
    if (proceed) {
        if (mFinalFrame.getTargetId() != getDeviceId()) {
            proceed = false;
        }
        anchorId = mFinalFrame.getSourceId();
        // skipping the check for anchorId against current mCurrentAnchorIndex to save processing time
    }
    if (proceed) {
        setMyState(MYSTATE_RECVD_VALID_FINAL);

        /* Retrieve Final transmission and Final response reception timestamps. */
        const uint64_t final_tx_ts = get_tx_timestamp_u64();
        const uint64_t final_response_rx_ts = get_rx_timestamp_u64();

        /* Get timestamps embedded in the Final response message. */
        uint32_t response_tx_ts, final_rx_ts, final_response_tx_ts;
        mFinalFrame.getTimestamps(&response_tx_ts, &final_rx_ts, &final_response_tx_ts);

        /* Compute time of flight. */
        const double Ra = (double)(final_rx_ts - response_tx_ts);
        const double Rb = (double)(final_response_rx_ts - final_tx_ts);
        const double Da = (double)(final_response_tx_ts - final_rx_ts);
        const double Db = (double)(final_tx_ts - mResponse_rx_ts);
        const int64_t tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
        const double tof = tof_dtu * DWT_TIME_UNITS;
        const double distance = tof * SPEED_OF_LIGHT;

        if (getMode() == UWB_MODE_ANT_DELAY_CALIBRATION) {
            // not checking distance plausibility when playing around with antenna delay
            mAntDelayCalibration.addDistanceMeasurement(distance);
        }

        const UwbMode mode = getMode();

        /* Retrieve anchor calculated TOF from Final frame. */
        double anchorCalculatedDistance = NAN;
        uint8_t fctCode;
        uint8_t fctData[FinalMsg::FINAL_DATA_SIZE];
        std::size_t actualFctDataLen;
        if (mFinalFrame.getFunctionCodeAndData(&fctCode, fctData, FinalMsg::FINAL_DATA_SIZE, &actualFctDataLen)) {
            anchorCalculatedDistance = ((double) ((int16_t) ((fctData[0] << 8) + fctData[1]))) / 100.0; // [cm] -> [m]
            if (!Location::isDistancePlausible(anchorCalculatedDistance) && (mode != UWB_MODE_ANT_DELAY_CALIBRATION)) {
                anchorCalculatedDistance = NAN;
            }
        }
        /* Display computed distance. */
        if (Location::isDistancePlausible(distance) || (mode == UWB_MODE_ANT_DELAY_CALIBRATION)) {
            if (mode == UWB_MODE_ANT_DELAY_CALIBRATION) {
                const double calibrationProgress = mAntDelayCalibration.getProgressPercent();
                const uint16_t antDelay = mAntDelayCalibration.getAntennaDelay();
                std::ostringstream msg;
                msg << std::fixed << std::setprecision(1) << +calibrationProgress
                    << "% #" << +(mAntDelayCalibrationResultPerRound.size()+1)
                    << "/" << +ANT_CALIB_MAX_ROUNDS
                    << " AntDelay=" << +antDelay
                    << " : dist to 0x" << std::hex << std::uppercase << +anchorId << std::dec
                    << ": " << std::setprecision(2) << +distance << "m (anchor " << +anchorCalculatedDistance << "m)";
                ESP_LOGW(TAG, "ANTDLY_CALIB %s", msg.str().c_str());
                sendLog(msg.str());
                if (mAntennaCalibrationProgressSensor != nullptr) {
                    const double totalProgress = ((double)mAntDelayCalibrationResultPerRound.size() * 100.0 + calibrationProgress)
                                                / ((double)ANT_CALIB_MAX_ROUNDS);
                    mAntennaCalibrationProgressSensor->publish_state((float)totalProgress);
                }

            } else {
                ESP_LOGW(TAG, "DIST anchor 0x%02X: %.2fm (from anchor %.2fm)",
                    anchorId, distance, anchorCalculatedDistance);
            }
            rangingDone(true, distance, anchorCalculatedDistance);
        } else {
            ESP_LOGW(TAG, "DIST anchor 0x%02X: %.2fm implausible (>%.0f)", anchorId, distance, Location::UWB_MAX_REACH_METER);
            rangingDone(false);
        }
    } else {
        const uint8_t anchorId = mAnchors.at(mCurrentAnchorIndex)->getId();
        ESP_LOGW(TAG, "0x%02X: recvdFrameFinal invalid", anchorId);
        setMyState(MYSTATE_RECVD_INVALID_FINAL);
    }
}

void UwbTagDevice::recvdValidResponse() {
    // should not come here
    ESP_LOGE(TAG, "entered recvdValidResponse() unexpectedly");
    // reset state machine
    setMyState(MY_DEFAULT_STATE);
}

void UwbTagDevice::recvdInvalidResponse() {
    rangingDone(false);
}

void UwbTagDevice::sentFinal() {
    /* remember when MYSTATE_WAIT_RECV_FINAL was entered for timeout calculation. */
    mEnteredWaitRecvFinalMicros = micros();
    setMyState(MYSTATE_WAIT_RECV_FINAL);
}

void UwbTagDevice::sendErrorFinal() {
    rangingDone(false);
}

void UwbTagDevice::recvdValidFinal() {
    // should not come here
    ESP_LOGE(TAG, "entered recvdValidFinal() unexpectedly");
    // reset state machine
    setMyState(MY_DEFAULT_STATE);
}

void UwbTagDevice::recvdInvalidFinal() {
    rangingDone(false);
}

void UwbTagDevice::rangingDone(bool success, double distance, double otherDistance) {
     // if running with high-frequency loop(), go back to normal frequency
    mHighFreqLoopRequester.stop();

    if (success) {
        mAnchorCurrentRangingSuccess[mCurrentAnchorIndex] = 0; // no more attempts
        const bool isCalibrating = getMode() == UWB_MODE_ANT_DELAY_CALIBRATION;
        if (!std::isnan(otherDistance)) {
            // Use the mean distance of the two distances
            const double meanDistance = (distance + otherDistance) / 2.0;
            // and set the error estimate as the difference between the mean and the original distance
            const double distanceErrEst = std::fabs(meanDistance - distance);
            (mAnchors[mCurrentAnchorIndex])->setDistance(meanDistance, distanceErrEst, isCalibrating);
        } else {
            (mAnchors[mCurrentAnchorIndex])->setDistance(distance, isCalibrating);
        }
    } else {
        const uint8_t attempts = mAnchorCurrentRangingSuccess[mCurrentAnchorIndex] -1;
        mAnchorCurrentRangingSuccess[mCurrentAnchorIndex] = attempts;
        if (attempts == 0) {
            const uint32_t now = millis();
            uint32_t lastDistanceMs;
            (mAnchors[mCurrentAnchorIndex])->getDistance(&lastDistanceMs);
            if (now - lastDistanceMs > MAX_AGE_ANCHOR_DISTANCE_MS) {
                // set anchor as 'away'
                (mAnchors[mCurrentAnchorIndex])->setDistance(NAN);
            }
        }
    }

    // next anchor
    setMyState(MYSTATE_WAIT_NEXT_ANCHOR_RANGING);
}

void UwbTagDevice::calculateLocationPrepare() {
    mLocationCalculationStartedMs = millis();
    if (mAnchors.size() > 1) {
        // collect all distances to anchors
        mAnchorPositionAndTagDistances.clear();
        for(const auto anchor: mAnchors) {
            AnchorPositionTagDistance anchorPosAndTagDist;
            anchorPosAndTagDist.anchorId = anchor->getId();
            anchorPosAndTagDist.anchorPosition.latitude =  anchor->getLatitude();
            anchorPosAndTagDist.anchorPosition.longitude = anchor->getLongitude();
            uint32_t millisDistance;
            anchorPosAndTagDist.tagDistance = anchor->getDistance(&millisDistance);
            if (Location::isValid(anchorPosAndTagDist)) {
                const uint32_t timeDiffMs = mLocationCalculationStartedMs - millisDistance;
                if (timeDiffMs <= MAX_AGE_ANCHOR_DISTANCE_MS) {
                    mAnchorPositionAndTagDistances.push_back(anchorPosAndTagDist);
                } else {
                    std::ostringstream msg;
                    msg << "anchor 0x" << HEX_TO_STREAM(2, anchorPosAndTagDist.anchorId)
                        << " timeDiffMs " << +timeDiffMs << " > " +MAX_AGE_ANCHOR_DISTANCE_MS;
                    ESP_LOGW(TAG, "%s", msg.str().c_str());
                    sendLog(msg.str());
                }
            } else {
                std::ostringstream msg;
                Location::LOG_ANCHOR_TO_STREAM(msg, anchorPosAndTagDist);
                ESP_LOGW(TAG, "anchor invalid %s", msg.str().c_str());
            }
        }

        const std::size_t anchorNum = mAnchorPositionAndTagDistances.size();
        if (anchorNum >= 2) { // min 2 anchors needed
            mLocationCalculationPhase = CALC_PHASE_INIT;
            setMyState(MYSTATE_CALCULATE_LOCATION_PHASES);
            return;
        }
    }
    // next ranging cycle
    setMyState(MYSTATE_WAIT_NEXT_RANGING_INTERVAL);
}

void UwbTagDevice::calculateLocationInPhases() {
    LatLong tagPosition;
    double errorEstimateMeters;
    CalcResult res;
    res = mLocation.calculatePosition(mLocationCalculationPhase, mAnchorPositionAndTagDistances, tagPosition, errorEstimateMeters);

    if (CALC_PHASE_OK == res) {
        return; // phase successful, next phase in next loop()
    } else
    if (CALC_OK == res) {
        // quick check against bounds of latitude and longitude
        if (Location::isValid(tagPosition)) {
            mTagPosition = tagPosition;
            mTagPositionErrorEstimate = errorEstimateMeters;
            // further post-processing in next loop
            setMyState(MYSTATE_CALCULATE_LOCATION_POST);
            return;
        } else {
            std::ostringstream msg;
            msg << "INVALID: " << FLOAT_TO_STREAM(7, tagPosition.latitude) << ","
                << FLOAT_TO_STREAM(7, tagPosition.longitude) << " errEst " << FLOAT_TO_STREAM(2, errorEstimateMeters) << "m";
            ESP_LOGW(TAG, "%s", msg.str().c_str());
            sendLog(msg.str());
        }
    } else {
        std::ostringstream msg;
        msg << "FAILED: " << toString(res);
        ESP_LOGW(TAG, "%s", msg.str().c_str());
        sendLog(msg.str());
    }
    // next ranging cycle
    setMyState(MYSTATE_WAIT_NEXT_RANGING_INTERVAL);
}

void UwbTagDevice::locationPostProcessing() {
    bool isLocationGood = false;
    float reportLatitude = NAN, reportLongitude = NAN, reportErrEst = NAN, reportAnchorNum = NAN;
    // check distances to anchors for plausibility
    bool isPositionNearAnchors = true;
    for(const auto anchor: mAnchors) {
        const LatLong anchorPosition = {anchor->getLatitude(), anchor->getLongitude()};
        const double distAnchor = Location::getHaversineDistance(mTagPosition, anchorPosition);
        if (!Location::isDistancePlausible(distAnchor)) {
            isPositionNearAnchors = false;
            std::ostringstream msg;
            msg << "calculated position " << FLOAT_TO_STREAM(7, mTagPosition.latitude) << ","
                << FLOAT_TO_STREAM(7, mTagPosition.longitude) << " errEst " << FLOAT_TO_STREAM(2, mTagPositionErrorEstimate)
                << " IMPLAUSIBLE dist " << FLOAT_TO_STREAM(2, distAnchor) << "m (>" << FLOAT_TO_STREAM(0, Location::UWB_MAX_REACH_METER)
                << " from anchor 0x" << HEX_TO_STREAM(2, anchor->getId());
            ESP_LOGW(TAG, "%s", msg.str().c_str());
            sendLog(msg.str());
            break;
        }
    }
    if (isPositionNearAnchors) {
        isLocationGood = true;
        reportLatitude = mTagPosition.latitude;
        reportLongitude = mTagPosition.longitude;
        reportErrEst = mTagPositionErrorEstimate;
        reportAnchorNum = (float)mAnchorPositionAndTagDistances.size();

        std::ostringstream msg;
        msg << "POSITION " << FLOAT_TO_STREAM(7, reportLatitude) << ","
            << FLOAT_TO_STREAM(7, reportLongitude) << " " << FLOAT_TO_STREAM(2, reportErrEst) << "m";
        ESP_LOGW(TAG, "%s", msg.str().c_str());
        sendLog(msg.str());
    }
    if (isLocationGood
       || (!isLocationGood && (mLocationCalculationStartedMs - mLastLocationReportMillis >= 30000U))) {
        if (!isLocationGood) {
            // invalidate previously reported location after 30s long failure period
            std::ostringstream msg;
            msg << "INVALIDATE position after " << +((mLocationCalculationStartedMs - mLastLocationReportMillis +500U)/1000U) << "s";
            ESP_LOGW(TAG, "%s", msg.str().c_str());
            sendLog(msg.str());
        }
        if (mLatitudeSensor != nullptr) {
            mLatitudeSensor->publish_state(reportLatitude);
        }
        if (mLongitudeSensor != nullptr) {
            mLongitudeSensor->publish_state(reportLongitude);
        }
        if (mLocationErrorEstimateSensor != nullptr) {
            mLocationErrorEstimateSensor->publish_state(reportErrEst);
        }
        if (mAnchorsInUseSensor != nullptr) {
            mAnchorsInUseSensor->publish_state(reportAnchorNum);
        }
        mLastLocationReportMillis = mLocationCalculationStartedMs;
    }
    // next ranging cycle
    setMyState(MYSTATE_WAIT_NEXT_RANGING_INTERVAL);
}

void UwbTagDevice::controlAntennaDelayCalibrationDistance(float distanceMeters) {
    if (mAntennaCalibrationDistanceNumber != nullptr) {
        if (getMode() != UWB_MODE_ANT_DELAY_CALIBRATION) {
            ESP_LOGW(TAG, "setCalibrationDistance %.2fm", distanceMeters);
            mAntDelayCalibration.setCalibrationDistance(distanceMeters);
            mAntennaCalibrationDistanceNumber->publish_state(distanceMeters); // confirm back
        } else {
            ESP_LOGE(TAG, "setCalibrationDistance %.2fm NOT possible while calibrating", distanceMeters);
            mAntennaCalibrationDistanceNumber->publish_state(AntDelayCalibration::DEFAULT_CALIBRATION_DISTANCE);
        }
    }
}

void UwbTagDevice::controlAntennaDelayCalibrationDevice(const std::string &device) {
    int idx = -1;
    if (getMode() != UWB_MODE_ANT_DELAY_CALIBRATION) {
        const long id = std::strtol(device.c_str(), nullptr, 16);
        for (idx = 0; idx < mAnchors.size(); idx++) {
            if (mAnchors.at(idx)->getId() == id) {
                ESP_LOGW(TAG, "setCalibrationDevice '%s'", device.c_str());
                mCurrentAntCalibAnchorIndex = idx;
                mAntDelayCalibDeviceSelect->publish_state(device); // confirm back
                break;
            }
        }
        if (idx >= mAnchors.size()) {
            ESP_LOGE(TAG, "setCalibrationDevice failed to find device '%s'", device.c_str());
        }
    }
    if (idx < 0 || idx >= mAnchors.size()) {
        // while calibration ongoing or device not found, report current device
        if (mCurrentAntCalibAnchorIndex >= 0 && mCurrentAntCalibAnchorIndex < mAnchors.size() && mAntDelayCalibDeviceSelect != nullptr) {
            const uint8_t id = mAnchors.at(mCurrentAntCalibAnchorIndex)->getId();
            std::ostringstream currentDevice;
            currentDevice << std::uppercase << std::hex << +id << std::dec;
            mAntDelayCalibDeviceSelect->publish_state(currentDevice.str());
        }
    }
}

void UwbTagDevice::pressedStartAntennaCalibration() {
    if (getMode() != UWB_MODE_ANT_DELAY_CALIBRATION) {
        if (mCurrentAntCalibAnchorIndex >= 0 && mCurrentAntCalibAnchorIndex < mAnchors.size()) {
            setMode(UWB_MODE_ANT_DELAY_CALIBRATION);
            setMyState(MY_DEFAULT_STATE);
        } else {
            ESP_LOGE(TAG, "select a calibration target device first");
            sendLog("select a calibration target device first");
        }
    } else {
        ESP_LOGE(TAG, "already in antenna delay calibration");
        sendLog("already in antenna delay calibration");
    }
}

}  // namespace uwb
}  // namespace esphome

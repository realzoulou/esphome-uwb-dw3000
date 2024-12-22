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

#pragma once

#include <cstdint>

#include "UwbListener.h"

#include "esphome/core/helpers.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include "dw3000_device_api.h"

namespace esphome {
namespace uwb {

/* Default antenna delay values for 64 MHz PRF */
const static uint16_t ANT_DLY_DEFAULT = 16385; // Decawave default

/* Macros for surrounding a code block which is time-critical and should not be interrupted. */
#define TIME_CRITICAL_START()
#define TIME_CRITICAL_END()

/* Macro for formatting hex numbers to streams. */
#define HEX_TO_STREAM(width, val) \
    std::hex << std::setw(width) << std::setfill('0') << std::uppercase \
    << +val << std::nouppercase << std::dec

/* Macro for formatting float numbers to streams. */
#define FLOAT_TO_STREAM(precision, val) \
    std::setprecision(precision) << +val

/* Diagnostic status */
typedef enum _DiagStatus {
    DIAG_UNKNOWN,
    DIAG_OK,
    DIAG_INIT_FAILED,
    DIAG_CONFIGURE_FAILED,
    DIAG_REBOOTING,
} DiagStatus;

typedef enum _UwbMode {
    UWB_MODE_RANGING,
    UWB_MODE_ANT_DELAY_CALIBRATION,
    UWB_MODE_ANT_DELAY_CALIBRATION_DONE,
} UwbMode;

class Dw3000Device {
public:
    /* buffer size for reading an incoming UWBMessage from DW3000 IC. */
    static const std::size_t UWB_RX_BUFFER_SIZE = 128;

    static uint8_t getNextTxSequenceNumberAndIncrease();
    static dwt_config_t* getConfig();

    explicit Dw3000Device();
    virtual ~Dw3000Device();

    virtual void setup();
    virtual void loop();

    uint8_t* getRxBuffer() const;
    uint16_t getRxBufferSize() const;

    inline void setDeviceId(const uint8_t id) { mDeviceId = id; }
    inline uint8_t getDeviceId() const { return mDeviceId; }

    inline void setAntennaDelay(const uint16_t antDelay) { mAntDelay = antDelay; }
    inline uint16_t getAntennaDelay() const { return mAntDelay; };

    inline void setLedsOffAfter(const uint32_t ledsOffAfterMs) { mLedsOffAfterMs = ledsOffAfterMs; }
    inline uint32_t getLedsOffAfter() const { return mLedsOffAfterMs; }

    inline void setVoltageSensor(sensor::Sensor* sensor) { mVoltageSensor = sensor; }
    inline void setTemperatureSensor(sensor::Sensor* sensor) { mTemperatureSensor = sensor; }
    inline void setDiagnosticStatusSensor(text_sensor::TextSensor* sensor) { mDiagnosticStatusSensor = sensor; }
    inline void setLogSensor(text_sensor::TextSensor* sensor) { mLogSensor = sensor; }

    virtual void sendLog(const std::string & str) const;

    virtual void maybeTurnLedsOff();
    virtual void maybeReportVoltageAndTemperature();

protected:
    void setDiagnosticStatus(DiagStatus status);
    inline DiagStatus getDiagnosticStatus() const { return mDiagStatus; }

    void setMode(const UwbMode mode);
    inline UwbMode getMode() const { return mUwbMode; }
    void setCalibrationAntennaDelays(const uint16_t delay) const;

protected:
    static const char* TAG;

    /* ID of this device. */
    uint8_t mDeviceId{0};

    /* Total amount of TX errors.*/
    uint32_t mTxErrorCount{0};

    /* Time in [ms] after startup to turn LEDs off, 0 keeps LEDs on). */
    uint32_t mLedsOffAfterMs{0};
    /* Remembers that maybeTurnLedsOff() has already done its job. */
    bool mLedsCheckDone{false};

    sensor::Sensor * mVoltageSensor{nullptr};
    sensor::Sensor * mTemperatureSensor{nullptr};
    /* last millis() when voltage and temperature sensor value reported. */
    uint32_t mVoltAndTempLastReportedMs{0};

    HighFrequencyLoopRequester mHighFreqLoopRequester;

private:
    static const char* diagStatusToString(const DiagStatus status);

private:
    static uint8_t txSequenceNumber;

    /* buffer for reading an incoming UWBMessage from DW3000 IC. */
    uint8_t* mRxBuffer{nullptr};

    /* TX/RX Antenna Delay. */
    uint16_t mAntDelay{ANT_DLY_DEFAULT};

    /* Mode: Ranging (default) or Antenna Delay Calibration. */
    UwbMode mUwbMode{UWB_MODE_RANGING};

    /* Diagnostic status. */
    DiagStatus mDiagStatus{DIAG_UNKNOWN};
    text_sensor::TextSensor * mDiagnosticStatusSensor{nullptr};

    /* Send logs to HA. */
    text_sensor::TextSensor * mLogSensor{nullptr};
};

}  // namespace uwb
}  // namespace esphome

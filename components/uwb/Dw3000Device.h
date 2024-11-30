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
#define TX_ANT_DLY  (16385) // Decawave default was confirmed to be good (by ranging with 8.0m distance)
#define RX_ANT_DLY  TX_ANT_DLY // for simplicity: RX = TX antenna delay

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
    static uint8_t getNextTxSequenceNumberAndIncrease();
    static dwt_config_t* getConfig();
    static uint8_t* getRxBuffer();
    static uint16_t getRxBufferSize();

    Dw3000Device();

    virtual void setup();
    virtual void loop();

    inline void setDeviceId(const uint8_t id) { mDeviceId = id; }
    inline uint8_t getDeviceId() const { return mDeviceId; }

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

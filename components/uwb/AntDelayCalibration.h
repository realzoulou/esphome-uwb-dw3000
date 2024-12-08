#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

namespace esphome {
namespace uwb {

class AntDelayCalibration {
public: // constants

    /* deviation LESS than this percentage of the target calibration distance is considered a good set of samples. */
    constexpr static double DEVIATION_PERCENT_GOOD = 8.0;
    /* start values for low and upper bounds of binary search. */
    constexpr static uint16_t LOW_ANTENNA_DELAY = 0;
    constexpr static uint16_t HI_ANTENNA_DELAY  = 32768;
    /* initial starting value for antenna delay. */
    const static uint16_t START_ANTENNA_DELAY;
    /* number of distance samples in a set. */
    constexpr static uint32_t DISTANCE_SAMPLES_PER_ANT_DELAY = 10;
    /* default target distance to reach. */
    constexpr static float DEFAULT_CALIBRATION_DISTANCE = 8;

public:
    AntDelayCalibration();

    /* Reset all run-time variables to prepare for a new antenna calibration. */
    void resetState();

    /* Set calibration distance to reach. */
    inline void setCalibrationDistance(const double distance) { mCalibrationDistance = distance; }
    /* Get calibration distance to reach. */
    inline double getCalibrationDistance() const { return mCalibrationDistance; }

    /* Add measured distance */
    void addDistanceMeasurement(const double measuredDistance);

    /* Get next antenna delay. */
    uint16_t getNextAntennaDelay();

    /* Is antenna delay calibration done. */
    bool isDone() const { return mCalibAntDelayDone; }

    /* Get current progress in percent (0.0 .. 100.0). Note: may not be monotonically increasing. */
    double getProgressPercent() const;

    /* Get current (if not yet isDone()) or calibrated (if isDone()) antenna delay. */
    uint16_t getAntennaDelay() { return mCalibAntDelay; }

    /* get standard deviation and mean value from array of samples. */
    static double getStandardDeviationAndMeanValue(const std::vector<double> & samples, double & outMeanValue);

protected:
    static const char* TAG;

private:
    /* target distance to reach. */
    double mCalibrationDistance{DEFAULT_CALIBRATION_DISTANCE};

    /* current distance. */
    double mCurrentDistance{NAN};

    /* set of distance samples. */
    std::vector<double> mDistancesPerAntDelay;

    /* While calibration ongoing this hold the current antenna delay being evaluated.
       If calibration done this holds the calibrated antenna delay value. */
    uint16_t mCalibAntDelay{START_ANTENNA_DELAY};

    bool mCalibAntDelayDone{false};

    /* binary search lower end of range. */
    uint16_t mLowerAntDelay{LOW_ANTENNA_DELAY};
    /* binary search upper end of range. */
    uint16_t mUpperAntDelay{HI_ANTENNA_DELAY};
    /* current step in binary search. */
    uint8_t mBinSearchStep{0};
};

}  // namespace uwb
}  // namespace esphome

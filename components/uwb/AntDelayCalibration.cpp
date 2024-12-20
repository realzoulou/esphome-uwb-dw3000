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

#include "AntDelayCalibration.h"

#include <assert.h>
#include <cmath>
#include <inttypes.h>
#include <iomanip>
#include <sstream>

#ifdef ESP32
#include "esphome/core/log.h"
    #define ANTDELAY_LOGW(ostringstream) ESP_LOGW(TAG, "%s", ostringstream.str().c_str());
    #define ANTDELAY_LOGI(ostringstream) ESP_LOGI(TAG, "%s", ostringstream.str().c_str());
#else
    #include <iostream>
    #define ANTDELAY_LOGW(ostringstream) (std::clog << ostringstream.str() << std::endl)
    #define ANTDELAY_LOGI(ostringstream) (std::cout << ostringstream.str() << std::endl)
#endif

namespace esphome {
namespace uwb {

const char* AntDelayCalibration::TAG = "antdelay";
const uint16_t AntDelayCalibration::START_ANTENNA_DELAY =
    (HI_ANTENNA_DELAY + LOW_ANTENNA_DELAY) / 2;

AntDelayCalibration::AntDelayCalibration() {
    mDistancesPerAntDelay.reserve(DISTANCE_SAMPLES_PER_ANT_DELAY);
}

void AntDelayCalibration::resetState() {
    mCalibAntDelay = START_ANTENNA_DELAY;
    mCalibAntDelayDone = false;
    mLowerAntDelay = LOW_ANTENNA_DELAY;
    mUpperAntDelay = HI_ANTENNA_DELAY;
    mDistancesPerAntDelay.clear();
    mBinSearchStep = 0;
}

void AntDelayCalibration::addDistanceMeasurement(const double measuredDistance) {
    if (mDistancesPerAntDelay.size() < DISTANCE_SAMPLES_PER_ANT_DELAY) {
        // while collecting samples with same antenna delay
        // store measured distance
        mDistancesPerAntDelay.push_back(measuredDistance);
#ifdef __UT_TEST__
        std::cout << "addDistanceMeasurement #" << +(mDistancesPerAntDelay.size()) << " : " << +measuredDistance << "m" << std::endl;
#endif
    }
}

uint16_t AntDelayCalibration::getNextAntennaDelay() {
    if (isDone()) {
        return mCalibAntDelay;
    }

    uint16_t nextAntennaDelay;
    if (mDistancesPerAntDelay.size() < DISTANCE_SAMPLES_PER_ANT_DELAY) {
        // While collecting samples with same antenna delay
        nextAntennaDelay = mCalibAntDelay;
    } else {
        // Collected a set of samples using same antenna delay.
        // Calculate standard deviation (always positive) and mean value (could be negative!) of set of measured distances
        double meanDistance;
        const double deviation = getStandardDeviationAndMeanValue(mDistancesPerAntDelay, meanDistance);
        std::ostringstream msg;
        msg << std::fixed << std::setprecision(3);
        msg << "currAntDelay " << +mCalibAntDelay << ": stddev " << +deviation << " mean " << +meanDistance << ": ";

        if (!std::isnan(deviation) && (deviation < ((DEVIATION_PERCENT_GOOD/100.0) * mCalibrationDistance))) {
            // deviation is good, accept the samples
            msg << "GOOD ";
            // Apply a modified binary search: Also halven the range,
            // but "distribute" the range 1:2 around the current antenna delay
            const uint16_t halvedRangeSize = (mUpperAntDelay - mLowerAntDelay) / 2;
            const uint16_t oneThird = halvedRangeSize / 3;
            if (meanDistance > mCalibrationDistance) {
                // mean measured distance is too large, increase the antenna delay
                mLowerAntDelay = mCalibAntDelay - oneThird;
                mUpperAntDelay = mCalibAntDelay + 2*oneThird;
                nextAntennaDelay = (mUpperAntDelay + mLowerAntDelay) / 2;
                // start new set of samples
                mDistancesPerAntDelay.clear();
                mBinSearchStep++;
                msg << "INC";
            } else if (meanDistance < mCalibrationDistance) {
                // mean measured distance is too small, decrease the antenna delay
                mLowerAntDelay = mCalibAntDelay - 2*oneThird;
                mUpperAntDelay = mCalibAntDelay + oneThird;
                nextAntennaDelay = (mUpperAntDelay + mLowerAntDelay) / 2;
                // start new set of samples
                mDistancesPerAntDelay.clear();
                mBinSearchStep++;
                msg << "DEC";
            } else {
                // exact match !?! wow... keep current antenna delay
                nextAntennaDelay = mCalibAntDelay;
                // no longer create new samples
                // caller is expected to check if calibration is done (true) and no longer call this function
                mCalibAntDelayDone = true;
                msg << "CALIB DONE";
            }
            // Note: upper and lower delay could be different by only 1 due to integer division (e.g. upper=16364 + lower=16363 / 2 = next=16363)
            // Then binary search cannot find any integer in between lower and upper anymore, so considered "same"
            const uint16_t diffUpperLower = mUpperAntDelay - mLowerAntDelay;
            if (std::abs(diffUpperLower) <= 1) { // use abs() just in case upper is actually smaller than lower, but should not happen
                // lower and upper delay are same: We are done
                nextAntennaDelay = mCalibAntDelay;
                mCalibAntDelayDone = true;
                msg << " CALIB DONE";
            }
            if (!mCalibAntDelayDone) {
                msg << " halvedRangeSize: " << +halvedRangeSize << " oneThird: " << +oneThird;
            }
        } else {
            msg << "BAD";
            // deviation is too big, ignore the samples and do another set of samples
            mDistancesPerAntDelay.clear();
            // keep current mBinSearchStep;
            // keep current antenna delay
            nextAntennaDelay = mCalibAntDelay;
        }
        ANTDELAY_LOGW(msg);

        std::ostringstream msg2;
        if (!mCalibAntDelayDone) {
            msg2 << " nextAntDelay: " << +nextAntennaDelay << " (" << +mLowerAntDelay << ".." << +mUpperAntDelay << ")";
        } else {
            msg2 << " ANTENNA CALIBRATION RESULT: " << +nextAntennaDelay;
        }
        ANTDELAY_LOGW(msg2);
    }
    mCalibAntDelay = nextAntennaDelay;
    return nextAntennaDelay;
}

/* static */
double AntDelayCalibration::getStandardDeviationAndMeanValue(const std::vector<double> & samples, double & outMeanValue) {
    if (samples.empty()) {
        outMeanValue = NAN;
        return NAN;
    }
    const double numSamples = (double) samples.size();
    double sumValues = 0.0;
    // Find the mean of all samples
    for (const double & s : samples) {
        sumValues += s;
    }
    const double meanValue = sumValues / numSamples;

    double sumOfSquares = 0.0;
    for (const double & s : samples) {
        // Find each sample's deviation from the mean
        const double sampleDeviationFromMean = s - meanValue;
        // Find the sum of squares
        sumOfSquares += (sampleDeviationFromMean * sampleDeviationFromMean);
    }
    // Find the variance: Divide the sum of the squares by the number of samples
    const double variance = sumOfSquares / numSamples;
    // Find the square root of the variance
    const double stdDev = std::sqrt(variance);

    outMeanValue = meanValue;
    return stdDev;
}

double AntDelayCalibration::getProgressPercent() const {
    if (isDone()) {
        return 100;
    }
    // We are using a binary search where the range of the antenna delay is halvened after having collected
    // a set of DISTANCE_SAMPLES_PER_ANT_DELAY samples.
    const double BIN_SEARCH_NUM = std::log2f((double)(HI_ANTENNA_DELAY - LOW_ANTENNA_DELAY));
    const double TOTAL_STEPS = BIN_SEARCH_NUM * (double) DISTANCE_SAMPLES_PER_ANT_DELAY;
    const double currentStep = ((double)mBinSearchStep * (double)DISTANCE_SAMPLES_PER_ANT_DELAY)
                                + (double)(mDistancesPerAntDelay.size());
    return (currentStep * 100) / TOTAL_STEPS; /* in percent */
}

}  // namespace uwb
}  // namespace esphome

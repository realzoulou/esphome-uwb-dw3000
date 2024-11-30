#include <gtest/gtest.h>

#include "AntDelayCalibration.h"

using namespace esphome::uwb;

////////////////////////////////////////////////////////
// AntDelayCalibration::getStandardDeviationAndMeanValue
TEST(AntDelayCalibration, getStandardDeviationAndMeanValue_emptyInput) {
    const std::vector<double> samples;
    double meanValue;

    EXPECT_TRUE(std::isnan(AntDelayCalibration::getStandardDeviationAndMeanValue(samples, meanValue)));
    EXPECT_TRUE(std::isnan(meanValue));
}

TEST(AntDelayCalibration, getStandardDeviationAndMeanValue_oneInputZero) {
    const std::vector<double> samples = { 0.0 };
    double meanValue;

    const double expStdDev = 0.0;
    const double expMeanValue = 0.0;
    EXPECT_EQ(expStdDev, AntDelayCalibration::getStandardDeviationAndMeanValue(samples, meanValue));
    EXPECT_EQ(expMeanValue, meanValue);
}

TEST(AntDelayCalibration, getStandardDeviationAndMeanValue_oneInputNan) {
    const std::vector<double> samples = { NAN };
    double meanValue;

    EXPECT_TRUE(std::isnan(AntDelayCalibration::getStandardDeviationAndMeanValue(samples, meanValue)));
    EXPECT_TRUE(std::isnan(meanValue));
}

TEST(AntDelayCalibration, getStandardDeviationAndMeanValue_oneValid) {
    const std::vector<double> samples = { 47.11 };
    double meanValue;

    const double expStdDev = 0.0;
    const double expMeanValue = 47.11;
    EXPECT_EQ(expStdDev, AntDelayCalibration::getStandardDeviationAndMeanValue(samples, meanValue));
    EXPECT_EQ(expMeanValue, meanValue);
}

TEST(AntDelayCalibration, getStandardDeviationAndMeanValue_twoValid) {
    const std::vector<double> samples = { 47.11, 42 };
    double meanValue;

    // https://www.calculator.net/standard-deviation-calculator.html?numberinputs=47.11%2C+42&ctype=p&x=Calculate
    const double expStdDev = 2.555; // Standard Deviation, σ: 2.555
    const double expMeanValue = 44.555; // Mean, μ:	44.555
    EXPECT_NEAR(expStdDev, AntDelayCalibration::getStandardDeviationAndMeanValue(samples, meanValue), 1e-3);
    EXPECT_NEAR(expMeanValue, meanValue, 1e-3);
}

TEST(AntDelayCalibration, getStandardDeviationAndMeanValue_threeValid) {
    const std::vector<double> samples = { 47.11, 42, -100.01 };
    double meanValue;

    // https://www.calculator.net/standard-deviation-calculator.html?numberinputs=47.11%2C+42%2C+-100.01&ctype=p&x=Calculate
    const double expStdDev = 68.180517419242; // Standard Deviation, σ: 68.180517419242
    const double expMeanValue = -3.6333333333333; // Mean, μ:	-3.6333333333333
    EXPECT_NEAR(expStdDev, AntDelayCalibration::getStandardDeviationAndMeanValue(samples, meanValue), 1e-12);
    EXPECT_NEAR(expMeanValue, meanValue, 1e-13);
}

TEST(AntDelayCalibration, getStandardDeviationAndMeanValue_threeValidAllSame) {
    const std::vector<double> samples = { 42, 42, 42 };
    double meanValue;
    // https://www.calculator.net/standard-deviation-calculator.html?numberinputs=42%2C42%2C42&ctype=p&x=Calculate
    const double expStdDev = 0; // Standard Deviation, σ: 0
    const double expMeanValue = 42; // Mean, μ:	42
    EXPECT_EQ(expStdDev, AntDelayCalibration::getStandardDeviationAndMeanValue(samples, meanValue));
    EXPECT_EQ(expMeanValue, meanValue);
}

//////////////////////////////////
// AntDelayCalibration constructor
TEST(AntDelayCalibration, constructor) {
    AntDelayCalibration antDelayCalib;
    EXPECT_FALSE(antDelayCalib.isDone());
    EXPECT_EQ(AntDelayCalibration::START_ANTENNA_DELAY, antDelayCalib.getAntennaDelay());
    const uint16_t expNextDelay = (AntDelayCalibration::HI_ANTENNA_DELAY - AntDelayCalibration::LOW_ANTENNA_DELAY)/2;
    EXPECT_EQ(expNextDelay, antDelayCalib.getNextAntennaDelay());
    EXPECT_EQ(0.0, antDelayCalib.getProgressPercent());
}

/////////////////////////////////////////////////
// AntDelayCalibration set/getCalibrationDistance
TEST(AntDelayCalibration, calibrationDistance) {
    const double dist = 47.11;
    AntDelayCalibration antDelayCalib;
    antDelayCalib.setCalibrationDistance(dist);
    EXPECT_EQ(dist, antDelayCalib.getCalibrationDistance());
}

/////////////////////////////////////////////////////////
// AntDelayCalibration addDistanceMeasurement/getNextAntennaDelay
TEST(AntDelayCalibration, addDistanceMeasurement_alwaysExactMatch) {
    AntDelayCalibration antDelayCalib;
    antDelayCalib.setCalibrationDistance(10.0);
    unsigned loopCnt = 0; // count loops to not end in endless loop
    const unsigned MAX_LOOP_CNT = AntDelayCalibration::DISTANCE_SAMPLES_PER_ANT_DELAY +1;
    while (!antDelayCalib.isDone() && loopCnt < MAX_LOOP_CNT) {
        loopCnt++;
        const uint16_t antDelay = antDelayCalib.getNextAntennaDelay();
        antDelayCalib.addDistanceMeasurement(10.0);
    }
    // expect to finish at exact DISTANCE_SAMPLES_PER_ANT_DELAY +1
    EXPECT_EQ(loopCnt, MAX_LOOP_CNT);
    EXPECT_TRUE(antDelayCalib.isDone());
    EXPECT_EQ(100, antDelayCalib.getProgressPercent());
    // with only 'perfect' measurements exactly reaching the calibration distance,
    // expected calibrated antenna delay is exactly at half of range
    const uint16_t expDelay = (AntDelayCalibration::HI_ANTENNA_DELAY - AntDelayCalibration::LOW_ANTENNA_DELAY)/2;
    EXPECT_EQ(expDelay, antDelayCalib.getAntennaDelay());
}

/* See AntDelayCalibrationExpectations.xlsx sheet 15999 */
TEST(AntDelayCalibration, calibrateTo15999) {
    const uint16_t expectedCalibratedAntennaDelay = 15999;
    AntDelayCalibration antDelayCalib;
    antDelayCalib.setCalibrationDistance(10.0);

    #define DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE \
        { 10.2, 9.8, 10.7, 11.5, 10.1, 10.0, 10.1, 9.9, 10.1, 10.1 } // mean 10.25, σ ~ 0.4738, so INC next delay
    #define DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE \
        { 9.7, 10.1, 9.9, 10.0, 9.9, 10.2, 9.9, 9.9, 10.1, 9.6 } // mean 9.93, σ ~ 0.1735, so DEC next delay

    const unsigned ROUNDS = 15; // 2^15 = 32768 = HI_ANTENNA_DELAY - LOW_ANTENNA_DELAY

    const double distanceInputs[ROUNDS][AntDelayCalibration::DISTANCE_SAMPLES_PER_ANT_DELAY] =
    {
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE
    };

    const uint16_t expNextAntennaDelay[] = {
        16384,
        13653,
        15018,
        15700,
        16041,
        15870,
        15955,
        15997,
        16018,
        16007,
        16002,
        15999,
        16000,
        15999,
        15999
    };
    static_assert(sizeof(expNextAntennaDelay) == ROUNDS * sizeof(uint16_t));

    double progress = antDelayCalib.getProgressPercent();
    EXPECT_EQ(0, antDelayCalib.getProgressPercent());
    unsigned r;
    for (r=0; r<ROUNDS; r++) {
        const bool isDone = antDelayCalib.isDone();
        EXPECT_FALSE(isDone);
        if (isDone) {
            std::cerr << ">> ABORT round #" << +(r+1) << " already isDone" << std::endl;
            return;
        }
        std::cout << ">> Round: " << +(r+1) << std::endl;
        for (unsigned i=0; i<AntDelayCalibration::DISTANCE_SAMPLES_PER_ANT_DELAY; i++) {
            double progress_ = antDelayCalib.getProgressPercent();
            EXPECT_EQ(progress_, progress);
            progress = progress_;
            const uint16_t nextAntennaDelay = antDelayCalib.getNextAntennaDelay();
            EXPECT_EQ(expNextAntennaDelay[r], nextAntennaDelay);
            if (expNextAntennaDelay[r] != nextAntennaDelay) {
                std::cerr << ">> ABORT round #" << +(r+1) << std::endl;
                return;
            }
            antDelayCalib.addDistanceMeasurement(distanceInputs[r][i]);
            progress_ = antDelayCalib.getProgressPercent();
            if (r == (ROUNDS-1)) {
                // in last round progress reaches 100%
                EXPECT_GE(progress_, progress);
            } else { // progress must monotonically increase
                EXPECT_GT(progress_, progress);
            }
            progress = progress_;
        }
    }
    EXPECT_EQ(r, ROUNDS);
    EXPECT_EQ(100, antDelayCalib.getProgressPercent());
    EXPECT_TRUE(antDelayCalib.isDone());
    EXPECT_EQ(expectedCalibratedAntennaDelay, antDelayCalib.getAntennaDelay());
}

/* See AntDelayCalibrationExpectations.xlsx sheet 15999 */
TEST(AntDelayCalibration, calibrateTo15999WithOneOutlierMeasurement) {
    const uint16_t expectedCalibratedAntennaDelay = 15999;
    AntDelayCalibration antDelayCalib;
    antDelayCalib.setCalibrationDistance(10.0);

    #define DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE \
        { 10.2, 9.8, 10.7, 11.5, 10.1, 10.0, 10.1, 9.9, 10.1, 10.1 } // mean 10.25, σ ~ 0.4738, so INC next delay
    #define DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE \
        { 9.7, 10.1, 9.9, 10.0, 9.9, 10.2, 9.9, 9.9, 10.1, 9.6 } // mean 9.93, σ ~ 0.1735, so DEC next delay
    #define DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE_EXCEEDS_DEVIATION \
        { 7.7, 10.1, 9.9, 10.0, 9.9, 10.2, 9.9, 9.9, 13.1, 8.6 } // mean 9.93, σ ~ 1.3 (>10m*8%), so BAD (keep delay)

    const unsigned ROUNDS = 16; // 1 more than 2^15 because 1 measurement set exceeds deviation

    const double distanceInputs[ROUNDS][AntDelayCalibration::DISTANCE_SAMPLES_PER_ANT_DELAY] =
    {
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE_EXCEEDS_DEVIATION,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_BELOW_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE,
        DISTANCE_SET_MEAN_ABOVE_CALIB_DISTANCE
    };

    const uint16_t expNextAntennaDelay[] = {
        16384,
        13653,
        15018,
        15018,
        15700,
        16041,
        15870,
        15955,
        15997,
        16018,
        16007,
        16002,
        15999,
        16000,
        15999,
        15999
    };
    static_assert(sizeof(expNextAntennaDelay) == ROUNDS * sizeof(uint16_t));

    double progress = antDelayCalib.getProgressPercent();
    EXPECT_EQ(0, antDelayCalib.getProgressPercent());
    unsigned r;
    for (r=0; r<ROUNDS; r++) {
        const bool isDone = antDelayCalib.isDone();
        EXPECT_FALSE(isDone);
        if (isDone) {
            std::cerr << ">> ABORT round #" << +(r+1) << " already isDone" << std::endl;
            return;
        }
        std::cout << ">> Round: " << +(r+1) << std::endl;
        for (unsigned i=0; i<AntDelayCalibration::DISTANCE_SAMPLES_PER_ANT_DELAY; i++) {
            double progress_ = antDelayCalib.getProgressPercent();
            EXPECT_EQ(progress_, progress);
            progress = progress_;
            const uint16_t nextAntennaDelay = antDelayCalib.getNextAntennaDelay();
            EXPECT_GT(antDelayCalib.getNextAntennaDelay(), progress);
            EXPECT_EQ(expNextAntennaDelay[r], nextAntennaDelay);
            if (expNextAntennaDelay[r] != nextAntennaDelay) {
                std::cerr << ">> ABORT round #" << +(r+1) << std::endl;
                return;
            }
            antDelayCalib.addDistanceMeasurement(distanceInputs[r][i]);
            progress_ = antDelayCalib.getProgressPercent();
            if (r == 3 && i == 0) {
                // last round had 'bad' entry in distanceInputs, expect that progress jumps back
                EXPECT_LT(progress_, progress);
            } else if (r == (ROUNDS-1)) {
                // in last round progress reaches 100%
                EXPECT_GE(progress_, progress);
            } else {  // progress must monotonically increase
                EXPECT_GT(progress_, progress);
            }
            progress = progress_;
        }
    }
    EXPECT_EQ(r, ROUNDS);
    EXPECT_EQ(100, antDelayCalib.getProgressPercent());
    EXPECT_TRUE(antDelayCalib.isDone());
    EXPECT_EQ(expectedCalibratedAntennaDelay, antDelayCalib.getAntennaDelay());
}

TEST(AntDelayCalibration, testSanitizersEnabled) {
    // uncomment for locally testing '-fsanitize=address' effectiveness
    // char* c = (char*) malloc(1023);
}

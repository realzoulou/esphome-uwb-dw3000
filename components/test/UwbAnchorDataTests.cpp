#include <gtest/gtest.h>

#include "UwbAnchorData.h"

#include <memory>

using namespace esphome::uwb;

#define TEST_ANCHOR_ID       (0xA1)
#define TEST_LATITUDE        (48.0)
#define TEST_LONGITUDE       (11.0)
#define TEST_MIN_DIST         (0.1)
#define TEST_MAX_SPEED        (1.0)

TEST(UwbAnchorData, constructor) {
    UwbAnchorData anchorData(TEST_ANCHOR_ID, TEST_LATITUDE, TEST_LONGITUDE, TEST_MIN_DIST, TEST_MAX_SPEED);
    std::unique_ptr<UwbAnchorData> pAnchorData = std::make_unique<UwbAnchorData>(TEST_ANCHOR_ID, TEST_LATITUDE, TEST_LONGITUDE, TEST_MIN_DIST, TEST_MAX_SPEED);
    UwbAnchorData &rAnchorData = *(pAnchorData.get());
    pAnchorData.reset();
}

TEST(UwbAnchorData, setSensor) {
    sensor::Sensor sensor;
    UwbAnchorData anchorData(TEST_ANCHOR_ID, TEST_LATITUDE, TEST_LONGITUDE, TEST_MIN_DIST, TEST_MAX_SPEED);
    anchorData.setSensor(&sensor);
    EXPECT_EQ(anchorData.getSensor(), &sensor);
}

TEST(UwbAnchorData, getId) {
    UwbAnchorData anchorData(TEST_ANCHOR_ID, TEST_LATITUDE, TEST_LONGITUDE, TEST_MIN_DIST, TEST_MAX_SPEED);
    EXPECT_EQ(anchorData.getId(), TEST_ANCHOR_ID);
}

TEST(UwbAnchorData, getLatLong) {
    UwbAnchorData anchorData(TEST_ANCHOR_ID, TEST_LATITUDE, TEST_LONGITUDE, TEST_MIN_DIST, TEST_MAX_SPEED);
    EXPECT_EQ(anchorData.getLatitude(), TEST_LATITUDE);
    EXPECT_EQ(anchorData.getLongitude(), TEST_LONGITUDE);
}

TEST(UwbAnchorData, distance) {
    UwbAnchorData anchorData(TEST_ANCHOR_ID, TEST_LATITUDE, TEST_LONGITUDE, TEST_MIN_DIST, TEST_MAX_SPEED);
    uint32_t millis;

    anchorData.setDistance(10);
    EXPECT_EQ(anchorData.getDistance(&millis), 10);
    EXPECT_GE(millis, 0);
    EXPECT_TRUE(std::isnan(anchorData.getDistanceErrorEstimate()));

    anchorData.setDistance(10.1, 0.1);
    EXPECT_EQ(anchorData.getDistance(&millis), 10.1);
    EXPECT_GE(millis, 0);
    EXPECT_EQ(anchorData.getDistanceErrorEstimate(), 0.1);

    anchorData.setDistance(20, 0.5); // because 20m exceeds TEST_MAX_SPEED, data is not taken over
    EXPECT_EQ(anchorData.getDistance(&millis), 10.1);
    EXPECT_EQ(anchorData.getDistanceErrorEstimate(), 0.1);
    EXPECT_GE(millis, 0);

    anchorData.setDistance(10.2, 0.3);
    EXPECT_EQ(anchorData.getDistance(&millis), 10.2);
    EXPECT_EQ(anchorData.getDistanceErrorEstimate(), 0.3);
    EXPECT_GE(millis, 0);
    EXPECT_EQ(anchorData.getDistance(nullptr), 10.2);

    anchorData.setDistance(NAN);
    EXPECT_TRUE(std::isnan(anchorData.getDistance(nullptr)));
    EXPECT_TRUE(std::isnan(anchorData.getDistanceErrorEstimate()));
}

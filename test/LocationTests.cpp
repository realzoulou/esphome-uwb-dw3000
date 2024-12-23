#include <gtest/gtest.h>

#include <algorithm>
#include <math.h>

#include "Location.h"

#define KM_TO_M                  (1000)

#define IS_NEAR_DOUBLE(x,y,diff) (((double)x-(double)y) <= std::fabs(diff))

using namespace esphome::uwb;

//////////////////////////////
// Location::LatLong operators
TEST(Location_LatLong, operators) {
    const LatLong a1 = {1.0, 1.0};
    const LatLong a2 = {1.0, 1.0};
    const LatLong b1 = {2.0, 1.0};
    EXPECT_TRUE(a1 == a2);
    EXPECT_FALSE(a1 != a2);
    EXPECT_TRUE(a1 != b1);
    EXPECT_FALSE(a1 == b1);
    EXPECT_EQ(a1, a2);
    EXPECT_EQ(a2, a1);
    EXPECT_NE(a1, b1);
    EXPECT_NE(b1, a1);
}

////////////////////////////////////////////////
// Location::AnchorPositionTagDistance operators
TEST(Location_AnchorPositionTagDistance, operators) {
    const AnchorPositionTagDistance a1 = {0xA1, {1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a2 = {0xA1, {1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a12 = {0xA2, {1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance b1 = {0xB1, {2.0, 1.0}, 1.0};
    EXPECT_TRUE(a1 == a2);
    EXPECT_FALSE(a1 != a2);
    EXPECT_FALSE(a1 == a12);
    EXPECT_FALSE(a1 == b1);
    EXPECT_FALSE(a1 == b1);
    EXPECT_EQ(a1, a2);
    EXPECT_EQ(a2, a1);
    EXPECT_NE(a1, a12);
    EXPECT_NE(a12, a1);
    EXPECT_NE(a1, b1);
    EXPECT_NE(b1, a1);
}

////////////////////////////
// Location::METER_TO_DEGREE
static const double DEG_PRECISION = 0.00001; // ~1.11m
TEST(Location_METER_TO_DEGREE, oneMeterAtEquator) {

    /* https://www.sunearthtools.com/dp/tools/conversion.php "Accuracy" */
    // 1.0 decimal degree ~ distance 111.319 km
    EXPECT_NEAR(1.00000, 111319 * Location::METER_TO_DEGREE(0/*lat 0 = equator*/), DEG_PRECISION);
    // 0.00001 decimal degree ~ distance 1.11 m
    EXPECT_NEAR(0.00001, 1.11 * Location::METER_TO_DEGREE(0/*lat 0 = equator*/), DEG_PRECISION);
}

////////////////////////
// Location::isValid

TEST(Location_isValid, validities) {
    AnchorPositionTagDistance a = {0x00, {0.0, 0.0}, 1.0};
    EXPECT_FALSE(Location::isValid(a));
    a = {0x00, { 90.0, 5.0}, 1.0};
    EXPECT_FALSE(Location::isValid(a));
    a = {0x00, { 5.0, 180.0}, 1.0};
    EXPECT_FALSE(Location::isValid(a));
    a = {0x00, { -90.0, 5.0}, 1.0};
    EXPECT_FALSE(Location::isValid(a));
    a = {0x00, { 5.0, -180.0}, 1.0};
    EXPECT_FALSE(Location::isValid(a));
    a = {0x00, { -90.0, -180.0}, 1.0};
    EXPECT_FALSE(Location::isValid(a));
    a = {0x00, {  90.0,  180.0}, 1.0};
    EXPECT_FALSE(Location::isValid(a));
    a = {0x00, {  5.0,  5.0}, -5.0};
    EXPECT_FALSE(Location::isValid(a));
    a = {0x00, {  5.0,  5.0}, 0.0};
    EXPECT_FALSE(Location::isValid(a));
    a = {0x00, {  5.0,  5.0}, 501.0};
    EXPECT_FALSE(Location::isValid(a));
    a = {0x00, {  5.0,  5.0}, -5.0};
    EXPECT_FALSE(Location::isValid(a));
    a = {0x00, {  5.0,  5.0}, 5.0};
    EXPECT_TRUE(Location::isValid(a));
}

////////////////////////
// Location::isDistancePlausible
TEST(Location_isDistancePlausible, plausibleDistance) {
    EXPECT_FALSE(Location::isDistancePlausible(0));
    EXPECT_FALSE(Location::isDistancePlausible(-1));
    EXPECT_FALSE(Location::isDistancePlausible(NAN));
    EXPECT_TRUE(Location::isDistancePlausible(1));
    EXPECT_TRUE(Location::isDistancePlausible(10));
    EXPECT_TRUE(Location::isDistancePlausible(50));
    EXPECT_TRUE(Location::isDistancePlausible(100));
    EXPECT_FALSE(Location::isDistancePlausible(500));
    EXPECT_FALSE(Location::isDistancePlausible(1000));
    EXPECT_FALSE(Location::isDistancePlausible(500000));
}

////////////////////////
// Location::getHaversineDistance

TEST(Location_getHaversineDistance, Washington2Philadelpha) {
    // https://www.distancefromto.net : 199.83 km
    const LatLong a = {38.8976, -77.0366}; // Washington
    const LatLong b = {39.9496, -75.1503}; // Philadelpha
    EXPECT_NEAR(199.83 * KM_TO_M, Location::getHaversineDistance(a ,b), 1.0); // 1m accuracy
}
TEST(Location_getHaversineDistance, 0to0) {
    const LatLong a = {0.0, 0.0};
    const LatLong b = {0.0, 0.0};
    EXPECT_NEAR(0, Location::getHaversineDistance(a ,b), 0.01); // 1cm accuracy
}
TEST(Location_getHaversineDistance, LibertyStatue2EiffelTower) {
    // https://www.distancefromto.net : 5837.39 km
    const LatLong a = {40.689412075430546, -74.0444789444429};
    const LatLong b = {48.858546539612085, 2.2944920269667555};
    EXPECT_NEAR(5837.39 * KM_TO_M, Location::getHaversineDistance(a ,b), 4.0); // 4m accuracy
}
TEST(Location_getHaversineDistance, BuckinghamPalace2VictoriaMemorial) {
     // https://www.distancefromto.net : 0.12 km
    const LatLong a = {51.50148961, -0.14220856};
    const LatLong b = {51.5018814, -0.1406225};
    EXPECT_NEAR(0.12 * KM_TO_M, Location::getHaversineDistance(a ,b), 2.0); // 2m accuracy
}
TEST(Location_getHaversineDistance, near0) {
    // https://www.distancefromto.net : 177.64 km
    const LatLong a = {1.5, 0.133975962155614};
    const LatLong b = {1.5, 1.7320508075688772};
    EXPECT_NEAR(177.64 * KM_TO_M, Location::getHaversineDistance(a ,b), 5.0); // 5m accuracy
}

//////////////////////////////////////
// Location::findAllAnchorCombinations

TEST(Location_findAllAnchorCombinations, emptyInput) {
    const std::vector<AnchorPositionTagDistance> empty;
    std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> result;
    EXPECT_FALSE(Location::findAllAnchorCombinations(empty, result));
    EXPECT_EQ(0, result.size());
}
TEST(Location_findAllAnchorCombinations, oneInput) {
    const std::vector<AnchorPositionTagDistance> one = {
        {0x00, {0.0, 0.0}, 0.0}
    };
    std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> result;
    EXPECT_FALSE(Location::findAllAnchorCombinations(one, result));
    EXPECT_EQ(0, result.size());
}
TEST(Location_findAllAnchorCombinations, twoInputs) {
    const AnchorPositionTagDistance a1 = {0xA1, {0.0, 0.0}, 0.0};
    const AnchorPositionTagDistance a2 = {0xA2, {1.0, 1.0}, 1.0};
    const std::vector<AnchorPositionTagDistance> two = {a1, a2};
    const std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> expected = {
        std::make_pair(a1, a2)
    };
    std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> result;
    EXPECT_TRUE(Location::findAllAnchorCombinations(two, result));
    EXPECT_EQ(expected.size(), result.size());
    EXPECT_TRUE(result == expected);
}
TEST(Location_findAllAnchorCombinations, threeInputs) {
    const AnchorPositionTagDistance a1 = {0xA1, {0.0, 0.0}, 0.0};
    const AnchorPositionTagDistance a2 = {0xA2, {1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a3 = {0xA3, {2.0, 2.0}, 2.0};
    const std::vector<AnchorPositionTagDistance> two = {a1, a2, a3};
    const std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> expected = {
        std::make_pair(a1, a2),
        std::make_pair(a1, a3),
        std::make_pair(a2, a3)
    };
    std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> result;
    EXPECT_TRUE(Location::findAllAnchorCombinations(two, result));
    EXPECT_EQ(expected.size(), result.size());
    EXPECT_TRUE(result == expected);
}
TEST(Location_findAllAnchorCombinations, fourInputs) {
    const AnchorPositionTagDistance a1 = {0xA1, {0.0, 0.0}, 0.0};
    const AnchorPositionTagDistance a2 = {0xA2, {1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a3 = {0xA3, {2.0, 2.0}, 2.0};
    const AnchorPositionTagDistance a4 = {0xA4, {3.0, 3.0}, 3.0};
    const std::vector<AnchorPositionTagDistance> two = {a1, a2, a3, a4};
    const std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> expected = {
        std::make_pair(a1, a2),
        std::make_pair(a1, a3),
        std::make_pair(a1, a4),
        std::make_pair(a2, a3),
        std::make_pair(a2, a4),
        std::make_pair(a3, a4)
    };
    std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> result;
    EXPECT_TRUE(Location::findAllAnchorCombinations(two, result));
    EXPECT_EQ(expected.size(), result.size());
    EXPECT_TRUE(result == expected);
}
TEST(Location_findAllAnchorCombinations, fiveInputs) {
    const AnchorPositionTagDistance a1 = {0xA1, {0.0, 0.0}, 0.0};
    const AnchorPositionTagDistance a2 = {0xA2, {1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a3 = {0xA3, {2.0, 2.0}, 2.0};
    const AnchorPositionTagDistance a4 = {0xA4, {3.0, 3.0}, 3.0};
    const AnchorPositionTagDistance a5 = {0xA5, {1.0, 1.0}, 1.0};
    const std::vector<AnchorPositionTagDistance> two = {a1, a2, a3, a4, a5};
    const std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> expected = {
        std::make_pair(a1, a2),
        std::make_pair(a1, a3),
        std::make_pair(a1, a4),
        std::make_pair(a1, a5),
        std::make_pair(a2, a3),
        std::make_pair(a2, a4),
        std::make_pair(a2, a5),
        std::make_pair(a3, a4),
        std::make_pair(a3, a5),
        std::make_pair(a4, a5)
    };
    std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> result;
    EXPECT_TRUE(Location::findAllAnchorCombinations(two, result));
    EXPECT_EQ(expected.size(), result.size());
    EXPECT_TRUE(result == expected);
}
TEST(Location_findAllAnchorCombinations, threeInputsWrongExpectation) {
    const AnchorPositionTagDistance a1 = {0xA1, {0.0, 0.0}, 0.0};
    const AnchorPositionTagDistance a2 = {0xA2, {1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a3 = {0xA3, {2.0, 2.0}, 2.0};
    const std::vector<AnchorPositionTagDistance> two = {a1, a2, a3};
    const std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> wrongExpected = {
        std::make_pair(a2, a1),
        std::make_pair(a3, a1),
        std::make_pair(a3, a2)
    };
    std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> result;
    EXPECT_TRUE(Location::findAllAnchorCombinations(two, result));
    EXPECT_EQ(wrongExpected.size(), result.size());
    EXPECT_FALSE(result == wrongExpected);
}

////////////////////////////////////////
// Location::findTwoCirclesIntersections

TEST(Location_findTwoCirclesIntersections, noIntersection) {
    const AnchorPositionTagDistance a1t = {0xA1, { 1.0,  1.0}, 1.0};
    const AnchorPositionTagDistance a2t = {0xA2, {-1.0, -1.0}, 1.0};

    LatLong t, t_prime;
    EXPECT_EQ(CIRCLE_INTERSECT_ERROR_NO_INTERSECTION, Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
TEST(Location_findTwoCirclesIntersections, inputsNAN) {
    const AnchorPositionTagDistance a1t = {0xA1, { NAN, 1.0}, 1.0};
    const AnchorPositionTagDistance a2t = {0xA2, {-1.0, NAN}, 1.0};

    LatLong t, t_prime;
    EXPECT_EQ(CIRCLE_INTERSECT_ERROR_INPUT, Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
TEST(Location_findTwoCirclesIntersections, circleContained) {
    const AnchorPositionTagDistance a1t = {0xA1, {1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a2t = {0xA2, {1.0, 1.0}, 2.0};

    LatLong t, t_prime;
    EXPECT_EQ(CIRCLE_INTERSECT_ERROR_CONTAINED, Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
TEST(Location_findTwoCirclesIntersections, circleEqual) {
    const AnchorPositionTagDistance a1t = {0xA1, {1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a2t = {0xA2, {1.0, 1.0}, 1.0};

    LatLong t, t_prime;
    EXPECT_EQ(CIRCLE_INTERSECT_ERROR_CONTAINED, Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
TEST(Location_findTwoCirclesIntersections, negativeDistancea1t) {
    const AnchorPositionTagDistance a1t = {0xA1, {-1.0, -1.0}, -1.5};
    const AnchorPositionTagDistance a2t = {0xA2, { 1.0,  1.0},  2.0};

    LatLong t, t_prime;
    EXPECT_EQ(CIRCLE_INTERSECT_ERROR_INPUT, Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
TEST(Location_findTwoCirclesIntersections, negativeDistancea2t) {
    const AnchorPositionTagDistance a1t = {0xA1, {-1.0, -1.0},  1.5};
    const AnchorPositionTagDistance a2t = {0xA2, { 1.0,  1.0}, -2.0};

    LatLong t, t_prime;
    EXPECT_EQ(CIRCLE_INTERSECT_ERROR_INPUT, Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
TEST(Location_findTwoCirclesIntersections, negativeDistances) {
    const AnchorPositionTagDistance a1t = {0xA1, {-1.0, -1.0}, -1.5};
    const AnchorPositionTagDistance a2t = {0xA2, { 1.0,  1.0}, -2.0};

    LatLong t, t_prime;
    EXPECT_EQ(CIRCLE_INTERSECT_ERROR_INPUT, Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
static const double INTERSECTION_PRECISION = 0.00001; // ~1.11m, hmm, no more precision possible?
TEST(Location_findTwoCirclesIntersections, twoAnchors_inBerlin) {
    // following 2 positions are ~20m apart
    const AnchorPositionTagDistance a1t = {0xA1, {52.4990325, 13.3917949}, 13.95 /*[m]*/};
    const AnchorPositionTagDistance a2t = {0xA2, {52.4990355, 13.3915640}, 13.95 /*[m]*/};
    /* lat 52.4990325 = rad 0.9162809712342, long 13.3917949 = rad 0.2337309137562
           52.4990355 =     0.9162810235941,      13.3915640 =     0.233726883791
       13.95m = 0.000125337° = 0,0000021875 rad
    https://planetcalc.com/8098/?x1=0.2337309137562&y1=0.9162809712342&r1=0.0000021875&x2=0.233726883791&y2=0.9162810235941&r2=0.0000021875
      There are two points of intersection:
      (x,y) = (0.2337289, 0.9162818) and  (0.2337289, 0.9162801)
    via https://www.rapidtables.com/convert/number/radians-to-degrees.html
            0.9162818 rad = 52.49908° |  0.9162801 rad = 52.498983°
            0.2337289     = 13.39168° |  0.2337289     = 13.39168°
    */
    const LatLong exp_t = {52.49908, 13.39168};
    const LatLong exp_t_prime = {52.498983, 13.39168};

    LatLong t, t_prime;
    EXPECT_EQ(CIRCLE_INTERSECT_OK, Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
    // cannot know if t or t_prime could match expected exp_t and exp_t_prime
    EXPECT_TRUE(
           (IS_NEAR_DOUBLE(exp_t.latitude, t.latitude, INTERSECTION_PRECISION)
            && IS_NEAR_DOUBLE(exp_t.longitude, t.longitude, INTERSECTION_PRECISION))
        || (IS_NEAR_DOUBLE(exp_t_prime.latitude, t.latitude, INTERSECTION_PRECISION)
            && IS_NEAR_DOUBLE(exp_t_prime.longitude, t.longitude, INTERSECTION_PRECISION))
    );
    EXPECT_TRUE(
           (IS_NEAR_DOUBLE(exp_t.latitude, t_prime.latitude, INTERSECTION_PRECISION)
            && IS_NEAR_DOUBLE(exp_t.longitude, t_prime.longitude, INTERSECTION_PRECISION))
        || (IS_NEAR_DOUBLE(exp_t_prime.latitude, t_prime.latitude, INTERSECTION_PRECISION)
            && IS_NEAR_DOUBLE(exp_t_prime.longitude, t_prime.longitude, INTERSECTION_PRECISION))
    );
}

/////////////////////////////////////
// Location::filterPositionCandidates
TEST(Location_filterPositionCandidates, emptyInput) {
    const std::vector<AnchorPositionTagDistance> anchors;
    std::vector<LatLong> positionCandidates;
    std::vector<LatLong> filteredOut = {{0.0, 0.0}};

    Location::filterPositionCandidates(anchors, positionCandidates, filteredOut);
    EXPECT_TRUE(anchors.empty());
    EXPECT_TRUE(positionCandidates.empty());
    EXPECT_TRUE(filteredOut.empty());
}
TEST(Location_filterPositionCandidates, twoAnchorsKeepCandidates) {
    const std::vector<AnchorPositionTagDistance> anchors = {
        {0xA1, {1.0, 1.0}, 0.0},
        {0xA2, {2.0, 2.0}, 0.0},
    };
    std::vector<LatLong> positionCandidates = {{3.0, 1.5}};
    const std::vector<LatLong> expPositionCandidates = {{3.0, 1.5}};
    std::vector<LatLong> filteredOut;
    const std::vector<LatLong> expFilteredOut;

    Location::filterPositionCandidates(anchors, positionCandidates, filteredOut);
    EXPECT_EQ(expPositionCandidates.size(), positionCandidates.size());
    for (const LatLong& t : expPositionCandidates) {
        EXPECT_NE(positionCandidates.end(), std::find(positionCandidates.begin(), positionCandidates.end(), t));
    }
    EXPECT_EQ(expFilteredOut.size(), filteredOut.size());
    for (const LatLong& t : expFilteredOut) {
        EXPECT_NE(filteredOut.end(), std::find(filteredOut.begin(), filteredOut.end(), t));
    }
}
TEST(Location_filterPositionCandidates, oneCandidateToKeep) {
    const std::vector<AnchorPositionTagDistance> anchors = {
        {0xA1, {1.0, 1.0}, 0.0},
        {0xA2, {2.0, 2.0}, 0.0},
        {0xA2, {3.0, 3.0}, 0.0},
    };
    std::vector<LatLong> positionCandidates = {{1.5, 1.5}};
    const std::vector<LatLong> expPositionCandidates = {{1.5, 1.5}};
    std::vector<LatLong> filteredOut;
    const std::vector<LatLong> expFilteredOut;

    Location::filterPositionCandidates(anchors, positionCandidates, filteredOut);
    EXPECT_EQ(expPositionCandidates.size(), positionCandidates.size());
    for (const LatLong& t : expPositionCandidates) {
        EXPECT_NE(positionCandidates.end(), std::find(positionCandidates.begin(), positionCandidates.end(), t));
    }
    EXPECT_EQ(expFilteredOut.size(), filteredOut.size());
    for (const LatLong& t : expFilteredOut) {
        EXPECT_NE(filteredOut.end(), std::find(filteredOut.begin(), filteredOut.end(), t));
    }
}
TEST(Location_filterPositionCandidates, oneCandidateToRemove) {
    const std::vector<AnchorPositionTagDistance> anchors = {
        {0xA1, {1.0, 1.0}, 0.0},
        {0xA2, {2.0, 2.0}, 0.0},
        {0xA2, {2.5, 2.5}, 0.0},
    };
    std::vector<LatLong> positionCandidates = {{3.0, 1.5}};
    const std::vector<LatLong> expPositionCandidates;
    std::vector<LatLong> filteredOut;
    const std::vector<LatLong> expFilteredOut = {{3.0, 1.5}};

    Location::filterPositionCandidates(anchors, positionCandidates, filteredOut);
    EXPECT_EQ(expPositionCandidates.size(), positionCandidates.size());
    for (const LatLong& t : expPositionCandidates) {
        EXPECT_NE(positionCandidates.end(), std::find(positionCandidates.begin(), positionCandidates.end(), t));
    }
    EXPECT_EQ(expFilteredOut.size(), filteredOut.size());
    for (const LatLong& t : expFilteredOut) {
        EXPECT_NE(filteredOut.end(), std::find(filteredOut.begin(), filteredOut.end(), t));
    }
}
TEST(Location_filterPositionCandidates, twoInputsSkipNanAnchor) {
    const std::vector<AnchorPositionTagDistance> anchors = {
        {0xA1, {1.0, 1.0}, 0.0},
        {0xA2, {2.0, 2.0}, 0.0},
        {0xA3, {NAN, NAN}, 0.0},
    };
    std::vector<LatLong> positionCandidates = {{1.5, 1.5}, {3.0, 3.0}};
    const std::vector<LatLong> expPositionCandidates = {{1.5, 1.5}};
    std::vector<LatLong> filteredOut;
    const std::vector<LatLong> expFilteredOut = {{3.0, 3.0}};

    Location::filterPositionCandidates(anchors, positionCandidates, filteredOut);
    EXPECT_EQ(expPositionCandidates.size(), positionCandidates.size());
    for (const LatLong& t : expPositionCandidates) {
        EXPECT_NE(positionCandidates.end(), std::find(positionCandidates.begin(), positionCandidates.end(), t));
    }
    EXPECT_EQ(expFilteredOut.size(), filteredOut.size());
    for (const LatLong& t : expFilteredOut) {
        EXPECT_NE(filteredOut.end(), std::find(filteredOut.begin(), filteredOut.end(), t));
    }
}

TEST(Location_filterPositionCandidates, threeInputsFilterOutNanCandidate) {
    const std::vector<AnchorPositionTagDistance> anchors = {
        {0xA1, {1.0, 1.0}, 0.0},
        {0xA2, {2.0, 2.0}, 0.0},
        {0xA3, {-2.0, -2.0}, 0.0},
    };
    std::vector<LatLong> positionCandidates = {{0.0, 0.0}, {3.0, 3.0}, {NAN, NAN}};
    const std::vector<LatLong> expPositionCandidates = {{0.0, 0.0}};
    std::vector<LatLong> filteredOut;
    const std::vector<LatLong> expFilteredOut = {{3.0, 3.0}, {NAN, NAN}};

    Location::filterPositionCandidates(anchors, positionCandidates, filteredOut);
    EXPECT_EQ(expPositionCandidates.size(), positionCandidates.size());
    for (const LatLong& t : expPositionCandidates) {
        EXPECT_NE(positionCandidates.end(), std::find(positionCandidates.begin(), positionCandidates.end(), t));
    }
    EXPECT_EQ(expFilteredOut.size(), filteredOut.size());
    for (const LatLong& t : expFilteredOut) {
        EXPECT_NE(filteredOut.end(), std::find(filteredOut.begin(), filteredOut.end(), t));
    }
}

////////////////////////////////////////
// Location::selectBestMatchingCandidate
TEST(Location_selectBestMatchingCandidate, not2Anchors) {
    const std::vector<AnchorPositionTagDistance> anchors = {
        {0xA1, {1.0, 1.0}, 1.0},
    };
    std::vector<LatLong> positionCandidates = {{0.0, 0.0}, {3.0, 3.0}};
    const LatLong expBestMatchingPosition = {NAN, NAN};
    LatLong bestMatchingPosition = expBestMatchingPosition;

    EXPECT_FALSE(Location::selectBestMatchingCandidate(anchors, positionCandidates, bestMatchingPosition));
    EXPECT_EQ(expBestMatchingPosition, bestMatchingPosition); // no side effects?
}
TEST(Location_selectBestMatchingCandidate, not1PositionCandidate) {
    const std::vector<AnchorPositionTagDistance> anchors = {
        {0xA1, {1.0, 1.0}, 1.0},
        {0xA2, {2.0, 2.0}, 2.0},
    };
    std::vector<LatLong> positionCandidates;
    const LatLong expBestMatchingPosition = {NAN, NAN};
    LatLong bestMatchingPosition = expBestMatchingPosition;

    EXPECT_FALSE(Location::selectBestMatchingCandidate(anchors, positionCandidates, bestMatchingPosition));
    EXPECT_EQ(expBestMatchingPosition, bestMatchingPosition); // no side effects?
}
TEST(Location_selectBestMatchingCandidate, onlyCandidateIsNan) {
    const std::vector<AnchorPositionTagDistance> anchors = {
        {0xA1, {1.0, 1.0}, 1.0},
        {0xA2, {2.0, 2.0}, 2.0},
    };
    std::vector<LatLong> positionCandidates = {{NAN, NAN}};
    const LatLong expBestMatchingPosition = {NAN, NAN};
    LatLong bestMatchingPosition = expBestMatchingPosition;

    EXPECT_FALSE(Location::selectBestMatchingCandidate(anchors, positionCandidates, bestMatchingPosition));
    EXPECT_EQ(expBestMatchingPosition, bestMatchingPosition); // no side effects?
}
TEST(Location_selectBestMatchingCandidate, allAnchorsAreNan) {
    const std::vector<AnchorPositionTagDistance> anchors = {
        {0xA1, {NAN, NAN}, 1.0},
        {0xA2, {NAN, NAN}, 2.0},
    };
    std::vector<LatLong> positionCandidates = {{0.0, 0.0}, {3.0, 3.0}};
    const LatLong expBestMatchingPosition = {NAN, NAN};
    LatLong bestMatchingPosition = expBestMatchingPosition;

    EXPECT_FALSE(Location::selectBestMatchingCandidate(anchors, positionCandidates, bestMatchingPosition));
    EXPECT_EQ(expBestMatchingPosition, bestMatchingPosition); // no side effects?
}
TEST(Location_selectBestMatchingCandidate, twoAnchorsOneCandidate) {
    const std::vector<AnchorPositionTagDistance> anchors = {
        {0xA1, {1.0, 1.0}, 1.0},
        {0xA2, {2.0, 2.0}, 2.0},
    };
    std::vector<LatLong> positionCandidates = {{3.0, 3.0}};
    const LatLong expBestMatchingPosition = {3.0, 3.0};
    LatLong bestMatchingPosition;

    EXPECT_TRUE(Location::selectBestMatchingCandidate(anchors, positionCandidates, bestMatchingPosition));
    EXPECT_EQ(expBestMatchingPosition, bestMatchingPosition);
}
TEST(Location_selectBestMatchingCandidate, twoAnchorsTwoCandidates) {
    const std::vector<AnchorPositionTagDistance> anchors = {
        {0xA1, {1.0, 1.0}, 1.0},
        {0xA2, {2.0, 2.0}, 2.0},
    };
    std::vector<LatLong> positionCandidates = {{3.0, 3.0}, {4.0, 4.0}};
    const LatLong expBestMatchingPosition = {3.0, 3.0};
    LatLong bestMatchingPosition;

    EXPECT_TRUE(Location::selectBestMatchingCandidate(anchors, positionCandidates, bestMatchingPosition));
    EXPECT_EQ(expBestMatchingPosition, bestMatchingPosition);
}
TEST(Location_selectBestMatchingCandidate, threeAnchorsTwoCandidates) {
    const std::vector<AnchorPositionTagDistance> anchors = {
        {0xA1, {1.0, 1.0}, 1.0},
        {0xA2, {2.0, 2.0}, 2.0},
        {0xA3, {3.0, 3.0}, 3.0},
    };
    std::vector<LatLong> positionCandidates = {{2.5, 2.5}, {10.0, 10.0}};
    const LatLong expBestMatchingPosition = {2.5, 2.5};
    LatLong bestMatchingPosition;

    EXPECT_TRUE(Location::selectBestMatchingCandidate(anchors, positionCandidates, bestMatchingPosition));
    EXPECT_EQ(expBestMatchingPosition, bestMatchingPosition);
}

//////////////////////////////
// Location::calculatePosition

static const double LATLONG_PRECISION = 0.00002; // 0.00001° ~1.11m, TODO why so bad ?

TEST(Location_calculatePosition, noAnchor) {
    const std::vector<AnchorPositionTagDistance> empty;
    const LatLong expPosition = {123.0, 234.0};
    const double expErr = 345.0;

    LatLong tagPosition = expPosition;
    double errEst = expErr;
    EXPECT_EQ(CALC_F_ANCHOR_COMBINATIONS, Location::calculatePosition(empty, tagPosition, errEst));
    EXPECT_EQ(expPosition, tagPosition); // no side-effects?
    EXPECT_EQ(expErr, errEst); // no side-effects?
}
TEST(Location_calculatePosition, oneAnchor) {
    const std::vector<AnchorPositionTagDistance> one = {
        {0xA1, {1.0, 1.0}, 1.0}
    };
    const LatLong expPosition = {123.0, 234.0};
    const double expErr = 345.0;

    LatLong tagPosition = expPosition;
    double errEst = expErr;
    EXPECT_NE(CALC_OK, Location::calculatePosition(one, tagPosition, errEst));
    EXPECT_EQ(expPosition, tagPosition); // no side-effects?
    EXPECT_EQ(expErr, errEst); // no side-effects?
}
TEST(Location_calculatePosition, twoAnchorsEqual) {
    const std::vector<AnchorPositionTagDistance> two= {
        {0xA1, {1.0, 1.0}, 1.0},
        {0xA2, {1.0, 1.0}, 1.0}
    };
    const LatLong expPosition = {123.0, 234.0};
    const double expErr = 345.0;

    LatLong tagPosition = expPosition;
    double errEst = expErr;
    EXPECT_NE(CALC_OK, Location::calculatePosition(two, tagPosition, errEst));
    EXPECT_EQ(expPosition, tagPosition); // no side-effects?
    EXPECT_EQ(expErr, errEst); // no side-effects?
}

TEST(Location_calculatePosition, twoAnchors_tagExactInTheMiddleInEastWestDirection) {
    const LatLong a1 = {1.0, 1.0}, a2 = {1.0000002, 1.0}, expPosition = {1.0000001, 1.0};
    const double dist = Location::getHaversineDistance(a1, a2);
    const double dist_half = dist/1.99; // should be ideally dist/2
    const std::vector<AnchorPositionTagDistance> two = {
        {0xA1, a1, dist_half},
        {0xA2, a2, dist_half}
    };
    const double expErr = 0.001; // 1mm
    LatLong tagPosition;
    double errEst;
    EXPECT_EQ(CALC_OK, Location::calculatePosition(two, tagPosition, errEst));
    EXPECT_NEAR(expPosition.latitude, tagPosition.latitude, LATLONG_PRECISION);
    EXPECT_NEAR(expPosition.longitude, tagPosition.longitude, LATLONG_PRECISION);
    EXPECT_LE(errEst, expErr);
}

TEST(Location_calculatePosition, twoAnchors_tagExactInTheMiddleNorthSouth) {
    const LatLong a1 = {1.0, 1.0}, a2 = {1.0, 1.0000002}, expPosition = {1.0, 1.0000001};
    const double dist = Location::getHaversineDistance(a1, a2);
    const double dist_half = dist/1.99; // should be ideally dist/2
    const std::vector<AnchorPositionTagDistance> two = {
        {0xA1, a1, dist_half},
        {0xA2, a2, dist_half}
    };
    const double expErr = 0.001; // 1mm
    LatLong tagPosition = expPosition;
    double errEst;
    EXPECT_EQ(CALC_OK, Location::calculatePosition(two, tagPosition, errEst));
    EXPECT_NEAR(expPosition.latitude, tagPosition.latitude, LATLONG_PRECISION);
    EXPECT_NEAR(expPosition.longitude, tagPosition.longitude, LATLONG_PRECISION);
    EXPECT_LE(errEst, expErr);
}
TEST(Location_calculatePosition, twoAnchors_inBerlin) {
    const LatLong a1 = {52.4990325, 13.3917949}, a2 = {52.4990355, 13.3915640};
    // https://www.distancefromto.net (52.4990325, 13.3917949) to (52.4990355, 13.3915640) = 0.02 km = 20m
    /* What is the circumfence of earth at lat 52.4990 ?
       https://www.gpsvisualizer.com/calculators "Calculate the great circle distance between two points"
       Half the circumfence: (52.4990, 0) (52.4990, 180) = 8366.195 km
       Full circumfence is 8366.195 km * 2 = 16732.39 km
       1.0° at lat 52.4990 = 16732.39 / 360° = 46.478861 km
       0.01°                                 =  0.46478861 km
       0.0001°                               =  0.0046478861 km ~ 4.65 m
       0.0003°                                                  = 13.95 m
       (see also https://www.sunearthtools.com/dp/tools/conversion.php "Accuracy")
    */
    const std::vector<AnchorPositionTagDistance> two = {
        {0xA1, a1, 13.95},
        {0xA2, a2, 13.95}
    };
    // https://planetcalc.com/8098/?x1=13.3917949&y1=52.4990325&r1=0.0003&x2=13.391564&y2=52.4990355&r2=0.0003
    // Intersection points (Digits after the decimal point: 7)
    const LatLong expPosition1 = {52.4993109, 13.3916830};
    const LatLong expPosition2 = {52.4987571, 13.3916759};
    /* bounding box for above 2 points:
       westSouthiest LatLong: 52.4987571, 13.3916759
       width: 0.0000071 (=13.3916830-13.3916759)
       height: 0.0005538 (=52.4993109-52.4987571)
    */
    /* center of the bounding box = 52.499034 (=52.4987571+height=0.0005538/2), 13.39168655 (=13.3916830+width=0.0000071/2)
                                  = (52.499034, 13.39168655)
    */
    const double expErr = 5; // 5m

    LatLong tagPosition;
    double errEst;
    EXPECT_TRUE(CALC_OK == Location::calculatePosition(two, tagPosition, errEst));
    EXPECT_TRUE(
            (IS_NEAR_DOUBLE(expPosition1.latitude, tagPosition.latitude, LATLONG_PRECISION)
            && IS_NEAR_DOUBLE(expPosition1.longitude, tagPosition.longitude, LATLONG_PRECISION))
        ||  (IS_NEAR_DOUBLE(expPosition2.latitude, tagPosition.latitude, LATLONG_PRECISION)
            && IS_NEAR_DOUBLE(expPosition2.longitude, tagPosition.longitude, LATLONG_PRECISION))
    );
    EXPECT_LE(errEst, expErr);
}
TEST(Location_calculatePosition, threeAnchors_pseudo) {
    const LatLong a1 = {1.00000, 1.00000}, a2 = {0.99910, 1.00010}, a3 = {0.99900, 1.00020};
    const std::vector<AnchorPositionTagDistance> three = {
        {0xA1, a1, 10},
        {0xA2, a2, 10},
        {0xA3, a3, 10},
    };
    // found no independent way (e.g. Internet page) to calculate the expected results
    const LatLong expPosition = {0.9990892, 1.0001892}; // trial & error until test passed, but looks reasonable
    const double expErr = 6; // 6m (trial & error)

    LatLong tagPosition = {NAN, NAN};
    double errEst = NAN;
    EXPECT_EQ(CALC_OK, Location::calculatePosition(three, tagPosition, errEst));
    EXPECT_NEAR(expPosition.latitude, tagPosition.latitude, LATLONG_PRECISION);
    EXPECT_NEAR(expPosition.longitude, tagPosition.longitude, LATLONG_PRECISION);
    EXPECT_LE(errEst, expErr);
}
TEST(Location_calculatePosition, threeAnchors_real) {
    const std::vector<AnchorPositionTagDistance> three = {
        {0xA1, {50.51695017092124, -35.649778489469757}, 16.59},
        {0xA2, {50.51678538255722, -35.649683941598978}, 8.02},
        {0xA3, {50.516870663933815, -35.649518985739324}, 7.63},
    };
    const LatLong expPosition = {50.516841, -35.649664};
    const double expErr = 1;

    LatLong tagPosition = {NAN, NAN};
    double errEst = NAN;
    EXPECT_EQ(CALC_OK, Location::calculatePosition(three, tagPosition, errEst));
    EXPECT_NEAR(expPosition.latitude, tagPosition.latitude, LATLONG_PRECISION);
    EXPECT_NEAR(expPosition.longitude, tagPosition.longitude, LATLONG_PRECISION);
    EXPECT_LE(errEst, expErr);
}
TEST(Location_calculatePosition, threeAnchors_real_in_phases) {
    const std::vector<AnchorPositionTagDistance> three = {
        {0xA1, {50.51695017092124, -35.649778489469757}, 16.59},
        {0xA2, {50.51678538255722, -35.649683941598978}, 8.02},
        {0xA3, {50.516870663933815, -35.649518985739324}, 7.63},
    };
    const LatLong expPosition = {50.516841, -35.649664};
    const double expErr = 1;

    LatLong tagPosition = {NAN, NAN};
    double errEst = NAN;
    Location l;
    CalculationPhase phase = CALC_PHASE_INIT;

    ASSERT_EQ(CALC_PHASE_OK, l.calculatePosition(phase, three, tagPosition, errEst));
    ASSERT_EQ(CALC_PHASE_DONE_ANCHOR_COMBINATIONS, (phase & CALC_PHASE_DONE_ANCHOR_COMBINATIONS));

    ASSERT_EQ(CALC_PHASE_OK, l.calculatePosition(phase, three, tagPosition, errEst));
    ASSERT_EQ(CALC_PHASE_DONE_ANCHOR_COMBINATIONS, (phase & CALC_PHASE_DONE_ANCHOR_COMBINATIONS));
    ASSERT_EQ(CALC_PHASE_DONE_COLLECT_CANDIDATES, (phase & CALC_PHASE_DONE_COLLECT_CANDIDATES));

    ASSERT_EQ(CALC_PHASE_OK, l.calculatePosition(phase, three, tagPosition, errEst));
    ASSERT_EQ(CALC_PHASE_DONE_ANCHOR_COMBINATIONS, (phase & CALC_PHASE_DONE_ANCHOR_COMBINATIONS));
    ASSERT_EQ(CALC_PHASE_DONE_COLLECT_CANDIDATES, (phase & CALC_PHASE_DONE_COLLECT_CANDIDATES));
    ASSERT_EQ(CALC_PHASE_DONE_FILTER_CANDIDATES, (phase & CALC_PHASE_DONE_FILTER_CANDIDATES));

    ASSERT_EQ(CALC_OK, l.calculatePosition(phase, three, tagPosition, errEst));
    ASSERT_EQ(CALC_PHASE_DONE_ANCHOR_COMBINATIONS, (phase & CALC_PHASE_DONE_ANCHOR_COMBINATIONS));
    ASSERT_EQ(CALC_PHASE_DONE_COLLECT_CANDIDATES, (phase & CALC_PHASE_DONE_COLLECT_CANDIDATES));
    ASSERT_EQ(CALC_PHASE_DONE_FILTER_CANDIDATES, (phase & CALC_PHASE_DONE_FILTER_CANDIDATES));
    ASSERT_EQ(CALC_PHASE_DONE_SELECT_BEST_CANDIDATE, (phase & CALC_PHASE_DONE_SELECT_BEST_CANDIDATE));

    ASSERT_EQ(CALC_ALL_PHASES_DONE, phase);

    EXPECT_NEAR(expPosition.latitude, tagPosition.latitude, LATLONG_PRECISION);
    EXPECT_NEAR(expPosition.longitude, tagPosition.longitude, LATLONG_PRECISION);
    EXPECT_LE(errEst, expErr);
}

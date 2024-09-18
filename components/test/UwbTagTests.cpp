#include <gtest/gtest.h>

#include <cmath>

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
}

////////////////////////////////////////////////
// Location::AnchorPositionTagDistance operators
TEST(Location_AnchorPositionTagDistance, operators) {
    const AnchorPositionTagDistance a1 = {{1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a2 = {{1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance b1 = {{2.0, 1.0}, 1.0};
    EXPECT_TRUE(a1 == a2);
    EXPECT_FALSE(a1 != a2);
    EXPECT_TRUE(a1 != b1);
    EXPECT_FALSE(a1 == b1);
}

////////////////////////////
// Location::METER_TO_DEGREE
static const double DEG_PRECISION = 0.00001; // "Precision choice is of 5 decimal place [...]."
TEST(Location_METER_TO_DEGREE, oneMeterAtEquator) {

    /* https://www.sunearthtools.com/dp/tools/conversion.php "Accuracy" */
    // 1.0 decimal degree ~ distance 111.319 km
    EXPECT_NEAR(1.00000, 111319 * Location::METER_TO_DEGREE(0/*lat 0 = equator*/), DEG_PRECISION);
    // 0.00001 decimal degree ~ distance 1.11 m
    EXPECT_NEAR(0.00001, 1.11 * Location::METER_TO_DEGREE(0/*lat 0 = equator*/), DEG_PRECISION);
}

////////////////////////
// Location::getDistance

TEST(Location_getDistance, Washington2Philadelpha) {
    // https://www.distancefromto.net : 199.83 km
    const LatLong a = {38.8976, -77.0366}; // Washington
    const LatLong b = {39.9496, -75.1503}; // Philadelpha
    EXPECT_NEAR(199.83 * KM_TO_M, Location::getDistance(a ,b), 1.0); // 1m accuracy
}
TEST(Location_getDistance, 0to0) {
    const LatLong a = {0.0, 0.0};
    const LatLong b = {0.0, 0.0};
    EXPECT_NEAR(0, Location::getDistance(a ,b), 0.01); // 1cm accuracy
}
TEST(Location_getDistance, LibertyStatue2EiffelTower) {
    // https://www.distancefromto.net : 5837.39 km
    const LatLong a = {40.689412075430546, -74.0444789444429};
    const LatLong b = {48.858546539612085, 2.2944920269667555};
    EXPECT_NEAR(5837.39 * KM_TO_M, Location::getDistance(a ,b), 4.0); // 4m accuracy
}
TEST(Location_getDistance, BuckinghamPalace2VictoriaMemorial) {
     // https://www.distancefromto.net : 0.12 km
    const LatLong a = {51.50148961, -0.14220856};
    const LatLong b = {51.5018814, -0.1406225};
    EXPECT_NEAR(0.12 * KM_TO_M, Location::getDistance(a ,b), 2.0); // 2m accuracy
}
TEST(Location_getDistance, near0) {
    // https://www.distancefromto.net : 177.64 km
    const LatLong a = {1.5, 0.133975962155614};
    const LatLong b = {1.5, 1.7320508075688772};
    EXPECT_NEAR(177.64 * KM_TO_M, Location::getDistance(a ,b), 5.0); // 5m accuracy
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
        {{0.0, 0.0}, 0.0}
    };
    std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> result;
    EXPECT_FALSE(Location::findAllAnchorCombinations(one, result));
    EXPECT_EQ(0, result.size());
}
TEST(Location_findAllAnchorCombinations, twoInputs) {
    const AnchorPositionTagDistance a1 = {{0.0, 0.0}, 0.0};
    const AnchorPositionTagDistance a2 = {{1.0, 1.0}, 1.0};
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
    const AnchorPositionTagDistance a1 = {{0.0, 0.0}, 0.0};
    const AnchorPositionTagDistance a2 = {{1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a3 = {{2.0, 2.0}, 2.0};
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
    const AnchorPositionTagDistance a1 = {{0.0, 0.0}, 0.0};
    const AnchorPositionTagDistance a2 = {{1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a3 = {{2.0, 2.0}, 2.0};
    const AnchorPositionTagDistance a4 = {{3.0, 3.0}, 3.0};
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
    const AnchorPositionTagDistance a1 = {{0.0, 0.0}, 0.0};
    const AnchorPositionTagDistance a2 = {{1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a3 = {{2.0, 2.0}, 2.0};
    const AnchorPositionTagDistance a4 = {{3.0, 3.0}, 3.0};
    const AnchorPositionTagDistance a5 = {{1.0, 1.0}, 1.0};
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
    const AnchorPositionTagDistance a1 = {{0.0, 0.0}, 0.0};
    const AnchorPositionTagDistance a2 = {{1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a3 = {{2.0, 2.0}, 2.0};
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
    const AnchorPositionTagDistance a1t = {{ 1.0,  1.0}, 1.0};
    const AnchorPositionTagDistance a2t = {{-1.0, -1.0}, 1.0};

    LatLong t, t_prime;
    EXPECT_FALSE(Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
TEST(Location_findTwoCirclesIntersections, inputsNAN) {
    const AnchorPositionTagDistance a1t = {{ NAN, 1.0}, 1.0};
    const AnchorPositionTagDistance a2t = {{-1.0, NAN}, 1.0};

    LatLong t, t_prime;
    EXPECT_FALSE(Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
TEST(Location_findTwoCirclesIntersections, circleContained) {
    const AnchorPositionTagDistance a1t = {{1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a2t = {{1.0, 1.0}, 2.0};

    LatLong t, t_prime;
    EXPECT_FALSE(Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
TEST(Location_findTwoCirclesIntersections, circleEqual) {
    const AnchorPositionTagDistance a1t = {{1.0, 1.0}, 1.0};
    const AnchorPositionTagDistance a2t = {{1.0, 1.0}, 1.0};

    LatLong t, t_prime;
    EXPECT_FALSE(Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
TEST(Location_findTwoCirclesIntersections, negativeDistancea1t) {
    const AnchorPositionTagDistance a1t = {{-1.0, -1.0}, -1.5};
    const AnchorPositionTagDistance a2t = {{ 1.0,  1.0},  2.0};

    LatLong t, t_prime;
    EXPECT_FALSE(Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
TEST(Location_findTwoCirclesIntersections, negativeDistancea2t) {
    const AnchorPositionTagDistance a1t = {{-1.0, -1.0},  1.5};
    const AnchorPositionTagDistance a2t = {{ 1.0,  1.0}, -2.0};

    LatLong t, t_prime;
    EXPECT_FALSE(Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
TEST(Location_findTwoCirclesIntersections, negativeDistances) {
    const AnchorPositionTagDistance a1t = {{-1.0, -1.0}, -1.5};
    const AnchorPositionTagDistance a2t = {{ 1.0,  1.0}, -2.0};

    LatLong t, t_prime;
    EXPECT_FALSE(Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
}
static const double INTERSECTION_PRECISION = 0.00001; // hmm, no more precision possible?
TEST(Location_findTwoCirclesIntersections, twoAnchors_inBerlin) {
    // following 2 positions are ~20m apart
    const AnchorPositionTagDistance a1t = {{52.4990325, 13.3917949}, 13.95 /*[m]*/};
    const AnchorPositionTagDistance a2t = {{52.4990355, 13.3915640}, 13.95 /*[m]*/};
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
    EXPECT_TRUE(Location::findTwoCirclesIntersections(a1t, a2t, t, t_prime));
    // cannot know if t or t_prime could match expected values
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

//////////////////////////////////
// Location::findBoundingRectangle
static const double RECT_PRECISION = 0.0000001;

TEST(Location_findBoundingRectangle, noPositions) {
    const std::vector<LatLong> positions;
    const BoundingRect expRect = {123.0, 234.0, 345.0, 456.0};
    BoundingRect rect = expRect;
    EXPECT_FALSE(Location::findBoundingRectangle(positions, rect));
    EXPECT_TRUE(expRect == rect); // no side-effects?
}
TEST(Location_findBoundingRectangle, onePosition) {
    const std::vector<LatLong> positions = {
        {1.0, 1.0}
    };
    const BoundingRect expRect = {{1.0, 1.0}, 0.0, 0.0};
    BoundingRect rect;
    EXPECT_TRUE(Location::findBoundingRectangle(positions, rect));
    EXPECT_NEAR(expRect.westSouthiest.latitude, rect.westSouthiest.latitude, RECT_PRECISION);
    EXPECT_NEAR(expRect.westSouthiest.longitude, rect.westSouthiest.longitude, RECT_PRECISION);
    EXPECT_NEAR(expRect.width, rect.width, RECT_PRECISION);
    EXPECT_NEAR(expRect.height, rect.height, RECT_PRECISION);
}
TEST(Location_findBoundingRectangle, twoPositions) {
    const std::vector<LatLong> positions = {
        {0.5, 2.0},
        {1.0, 1.0}
    };
    const BoundingRect expRect = {{0.5, 1.0}, 1.0, 0.5};
    BoundingRect rect;
    EXPECT_TRUE(Location::findBoundingRectangle(positions, rect));
    EXPECT_NEAR(expRect.westSouthiest.latitude, rect.westSouthiest.latitude, RECT_PRECISION);
    EXPECT_NEAR(expRect.westSouthiest.longitude, rect.westSouthiest.longitude, RECT_PRECISION);
    EXPECT_NEAR(expRect.width, rect.width, RECT_PRECISION);
    EXPECT_NEAR(expRect.height, rect.height, RECT_PRECISION);
}
TEST(Location_findBoundingRectangle, twoPositionsSomeNeg) {
    const std::vector<LatLong> positions = {
        {0.75, -1.5},
        {0.25, -0.5}
    };
    const BoundingRect expRect = {{0.25, -1.5}, 1.0, 0.5};
    BoundingRect rect;
    EXPECT_TRUE(Location::findBoundingRectangle(positions, rect));
    EXPECT_NEAR(expRect.westSouthiest.latitude, rect.westSouthiest.latitude, RECT_PRECISION);
    EXPECT_NEAR(expRect.westSouthiest.longitude, rect.westSouthiest.longitude, RECT_PRECISION);
    EXPECT_NEAR(expRect.width, rect.width, RECT_PRECISION);
    EXPECT_NEAR(expRect.height, rect.height, RECT_PRECISION);
}
TEST(Location_findBoundingRectangle, twoPositionsEqual) {
    const std::vector<LatLong> positions = {
        {-0.8, 0.8},
        {-0.8, 0.8}
    };
    const BoundingRect expRect = {{-0.8, 0.8}, 0.0, 0.0}; // rectangle is a point!
    BoundingRect rect;
    EXPECT_TRUE(Location::findBoundingRectangle(positions, rect));
    EXPECT_NEAR(expRect.westSouthiest.latitude, rect.westSouthiest.latitude, RECT_PRECISION);
    EXPECT_NEAR(expRect.westSouthiest.longitude, rect.westSouthiest.longitude, RECT_PRECISION);
    EXPECT_NEAR(expRect.width, rect.width, RECT_PRECISION);
    EXPECT_NEAR(expRect.height, rect.height, RECT_PRECISION);
}
TEST(Location_findBoundingRectangle, twoPositionsNAN) {
    const std::vector<LatLong> positions = {
        {NAN, NAN},
        {NAN, NAN}
    };
    BoundingRect rect;
    EXPECT_FALSE(Location::findBoundingRectangle(positions, rect));
}
TEST(Location_findBoundingRectangle, threePositions) {
    const std::vector<LatLong> positions = {
        {0.75, -1.5}, // from twoPositionsSomeNeg
        {0.25, -0.5}, // from twoPositionsSomeNeg
        {0.50, -1.0}, // a point inside above 2 points BoundingRectangle
    };
    const BoundingRect expRect = {{0.25, -1.5}, 1.0, 0.5}; // same as twoPositionsSomeNeg
    BoundingRect rect;
    EXPECT_TRUE(Location::findBoundingRectangle(positions, rect));
    EXPECT_NEAR(expRect.westSouthiest.latitude, rect.westSouthiest.latitude, RECT_PRECISION);
    EXPECT_NEAR(expRect.westSouthiest.longitude, rect.westSouthiest.longitude, RECT_PRECISION);
    EXPECT_NEAR(expRect.width, rect.width, RECT_PRECISION);
    EXPECT_NEAR(expRect.height, rect.height, RECT_PRECISION);
}

//////////////////////////////
// Location::calculatePosition

static const double LATLONG_PRECISION = 0.00001; // TODO why so bad ?

TEST(Location_calculatePosition, noAnchor) {
    const std::vector<AnchorPositionTagDistance> empty;
    const LatLong expPosition = {123.0, 234.0};
    const double expErr = 345.0;

    LatLong tagPosition = expPosition;
    double errEst = expErr;
    EXPECT_FALSE(Location::calculatePosition(empty, tagPosition, errEst));
    EXPECT_EQ(expPosition, tagPosition); // no side-effects?
    EXPECT_EQ(expErr, errEst); // no side-effects?
}
TEST(Location_calculatePosition, oneAnchor) {
    const std::vector<AnchorPositionTagDistance> one = {
        {{1.0, 1.0}, 1.0}
    };
    const LatLong expPosition = {123.0, 234.0};
    const double expErr = 345.0;

    LatLong tagPosition = expPosition;
    double errEst = expErr;
    EXPECT_FALSE(Location::calculatePosition(one, tagPosition, errEst));
    EXPECT_EQ(expPosition, tagPosition); // no side-effects?
    EXPECT_EQ(expErr, errEst); // no side-effects?
}
TEST(Location_calculatePosition, twoAnchorsEqual) {
    const std::vector<AnchorPositionTagDistance> two= {
        {{1.0, 1.0}, 1.0},
        {{1.0, 1.0}, 1.0}
    };
    const LatLong expPosition = {123.0, 234.0};
    const double expErr = 345.0;

    LatLong tagPosition = expPosition;
    double errEst = expErr;
    EXPECT_FALSE(Location::calculatePosition(two, tagPosition, errEst));
    EXPECT_EQ(expPosition, tagPosition); // no side-effects?
    EXPECT_EQ(expErr, errEst); // no side-effects?
}

TEST(Location_calculatePosition, twoAnchors_tagExactInTheMiddleInEastWestDirection) {
    const LatLong a1 = {1.0, 1.0}, a2 = {1.0000002, 1.0}, expPosition = {1.0000001, 1.0};
    const double dist = Location::getDistance(a1, a2);
    const double dist_half = dist/1.99; // should be ideally dist/2
    const std::vector<AnchorPositionTagDistance> two = {
        {a1, dist_half},
        {a2, dist_half}
    };
    const double expErr = 0.001; // 1mm
    LatLong tagPosition;
    double errEst;
    EXPECT_TRUE(Location::calculatePosition(two, tagPosition, errEst));
    EXPECT_NEAR(expPosition.latitude, tagPosition.latitude, LATLONG_PRECISION);
    EXPECT_NEAR(expPosition.longitude, tagPosition.longitude, LATLONG_PRECISION);
    EXPECT_NEAR(expErr, errEst, 0.01); // 1cm accuracy
}

TEST(Location_calculatePosition, twoAnchors_tagExactInTheMiddleNorthSouth) {
    const LatLong a1 = {1.0, 1.0}, a2 = {1.0, 1.0000002}, expPosition = {1.0, 1.0000001};
    const double dist = Location::getDistance(a1, a2);
    const double dist_half = dist/1.99; // should be ideally dist/2
    const std::vector<AnchorPositionTagDistance> two = {
        {a1, dist_half},
        {a2, dist_half}
    };
    const double expErr = 0.001; // 1mm
    LatLong tagPosition = expPosition;
    double errEst;
    EXPECT_TRUE(Location::calculatePosition(two, tagPosition, errEst));
    EXPECT_NEAR(expPosition.latitude, tagPosition.latitude, LATLONG_PRECISION);
    EXPECT_NEAR(expPosition.longitude, tagPosition.longitude, LATLONG_PRECISION);
    EXPECT_NEAR(expErr, errEst, 0.01); // 1cm accuracy
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
        {a1, 13.95},
        {a2, 13.95}
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
    EXPECT_TRUE(Location::calculatePosition(two, tagPosition, errEst));
    EXPECT_TRUE(
            (IS_NEAR_DOUBLE(expPosition1.latitude, tagPosition.latitude, LATLONG_PRECISION)
            && IS_NEAR_DOUBLE(expPosition1.longitude, tagPosition.longitude, LATLONG_PRECISION))
        ||  (IS_NEAR_DOUBLE(expPosition2.latitude, tagPosition.latitude, LATLONG_PRECISION)
            && IS_NEAR_DOUBLE(expPosition2.longitude, tagPosition.longitude, LATLONG_PRECISION))
    );
    EXPECT_NEAR(expErr, errEst, 1); // 1m accuracy
}
TEST(Location_calculatePosition, threeAnchors) {
    const LatLong a1 = {1.00000, 1.00000}, a2 = {0.99910, 1.00010}, a3 = {0.99900, 1.00020};
    const std::vector<AnchorPositionTagDistance> three = {
        {a1, 10},
        {a2, 10},
        {a3, 10},
    };
    // found no independent way (e.g. Internet page) to calculate the expected results
    const LatLong expPosition = {0.99905, 1.00015}; // trial & error until test passed, but looks reasonable
    const double expErr = 6; // 6m (trial & error)

    LatLong tagPosition;
    double errEst;
    EXPECT_TRUE(Location::calculatePosition(three, tagPosition, errEst));
    EXPECT_NEAR(expPosition.latitude, tagPosition.latitude, LATLONG_PRECISION);
    EXPECT_NEAR(expPosition.longitude, tagPosition.longitude, LATLONG_PRECISION);
    EXPECT_NEAR(expErr, errEst, 1); // 1m accuracy
}
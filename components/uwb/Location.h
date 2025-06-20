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

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdint>
#include <sstream>
#include <vector>

namespace esphome {
namespace uwb {

typedef struct sLatLong {
    double latitude;  // degree
    double longitude; // degree
} LatLong;

typedef struct sLatLongAlt {
    double latitude;  // degree
    double longitude; // degree
    double altitude; // meter above the WGS84 reference ellipsoid
} LatLongAlt;

typedef struct sXYZ {
    double x; // meter
    double y; // meter
    double z; // meter
} XYZ;
typedef XYZ ECEF; // Earth-centered, Earth-fixed coordinate system [m]
typedef XYZ ENU; // East/North/Up [m]

typedef enum {
    X_AXIS,
    Y_AXIS,
    Z_AXIS
} Axis;

inline static bool operator==(const LatLong& lhs, const LatLong& rhs) {
    return (lhs.latitude == rhs.latitude && lhs.longitude == rhs.longitude)
        // allow that NAN == NAN (other than the standard!)
        || (std::isnan(lhs.latitude) && std::isnan(rhs.latitude))
        || (std::isnan(lhs.longitude) && std::isnan(rhs.longitude));
}
inline static bool operator!=(const LatLong& lhs, const LatLong& rhs) {
    return !(lhs==rhs);
}
typedef struct sAnchorPositionTagDistance{
    uint8_t anchorId;
    LatLong anchorPosition;
    double tagDistance;
    double tagDistanceErrEstimate;
} AnchorPositionTagDistance;

inline static bool operator==(const AnchorPositionTagDistance& lhs, const AnchorPositionTagDistance& rhs) {
    return lhs.anchorId == rhs.anchorId
        && lhs.anchorPosition == rhs.anchorPosition
        && ( (lhs.tagDistance == rhs.tagDistance)
          // allow that NAN == NAN (other than the standard!)
          || (std::isnan(lhs.tagDistance) && std::isnan(rhs.tagDistance) ) )
        && ( (lhs.tagDistanceErrEstimate == rhs.tagDistanceErrEstimate)
          // allow that NAN == NAN (other than the standard!)
          || (std::isnan(lhs.tagDistanceErrEstimate) && std::isnan(rhs.tagDistanceErrEstimate) ) );
}
inline static bool operator!=(const AnchorPositionTagDistance& lhs, const AnchorPositionTagDistance& rhs) {
    return !(lhs==rhs);
}

typedef enum eCalcResult {
    CALC_OK = 0,
    CALC_PHASE_OK = 5,
    CALC_F_ANCHOR_COMBINATIONS = 10,
    CALC_F_NO_CANDIDATES = 20,
    CALC_F_BEST_MATCH = 30,
} CalcResult;

inline static const char* toString(const CalcResult & res) {
    switch(res) {
        case CALC_OK: return "OK";
        case CALC_F_ANCHOR_COMBINATIONS: return "not enough anchor combinations";
        case CALC_F_NO_CANDIDATES: return "no candidates";
        case CALC_F_BEST_MATCH: return "no best matching position candidate";
        default: return "?!?";
    }
}

typedef enum eCircleIntersectionResult {
    CIRCLE_INTERSECT_OK,
    CIRCLE_INTERSECT_ERROR_INPUT,
    CIRCLE_INTERSECT_ERROR_NO_INTERSECTION,
    CIRCLE_INTERSECT_ERROR_CONTAINED,
} CircleIntersectionResult;

inline static const char* toString(const CircleIntersectionResult & res) {
    switch(res) {
        case CIRCLE_INTERSECT_OK: return "OK";
        case CIRCLE_INTERSECT_ERROR_INPUT: return "input error";
        case CIRCLE_INTERSECT_ERROR_NO_INTERSECTION: return "circles do not intersect";
        case CIRCLE_INTERSECT_ERROR_CONTAINED: return "circle contained in each other";
        default: return "?!?";
    }
}

typedef uint8_t CalculationPhase;
constexpr static CalculationPhase CALC_RUN_ALL_PHASES_AT_ONCE            = 0x01;
constexpr static CalculationPhase CALC_PHASE_INIT                        = 0x80;
constexpr static CalculationPhase CALC_PHASE_DONE_ANCHOR_COMBINATIONS    = 0x02;
constexpr static CalculationPhase CALC_PHASE_DONE_COLLECT_CANDIDATES     = 0x04;
constexpr static CalculationPhase CALC_PHASE_DONE_FILTER_CANDIDATES      = 0x08;
constexpr static CalculationPhase CALC_PHASE_DONE_SELECT_BEST_CANDIDATE  = 0x10;
constexpr static CalculationPhase CALC_ALL_PHASES_DONE =
      CALC_PHASE_DONE_ANCHOR_COMBINATIONS
    + CALC_PHASE_DONE_COLLECT_CANDIDATES
    + CALC_PHASE_DONE_FILTER_CANDIDATES
    + CALC_PHASE_DONE_SELECT_BEST_CANDIDATE;

class Location {

public: // constants
    constexpr static double UWB_MAX_REACH_METER = 327.67; // 32767 cm

public: // static methods
    static bool isValid(const AnchorPositionTagDistance & a);
    static bool isValid(const LatLong & a);
    inline static bool isDistancePlausible(const double distance) {
        return (!std::isnan(distance) && (distance> 0) && (distance <= UWB_MAX_REACH_METER));
    }
    static void LOG_ANCHOR_TO_STREAM(std::ostringstream & ostream, const AnchorPositionTagDistance & anchor);
    static void LOG_ANCHORS_TO_STREAM(std::ostringstream & ostream, const std::vector<AnchorPositionTagDistance> & anchors);

    static CalcResult calculatePosition(const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                        LatLong & outputTagPosition, double & outputTagPositionErrorEstimate);

    /* get squerical distance between 2 points on earth in [m] using Haversine formula. */
    static double getHaversineDistance(const LatLong & from, const LatLong & to);

    /* convert WGS-84 geodetic datum to East/North/Up ENU relative to reference geodetic datum. */
    static void latLongToEnu(const LatLongAlt & latLong, const LatLongAlt & refLatLong, ENU & enu);
    /* convert a local East/North/Up ENU to to WGS-84 geodetic datum with reference WGS-84 geodetic datu. */
    static bool enuToLatLong(const ENU & enu, const LatLongAlt & refLatLong, LatLongAlt & latLong);

    /* find all distinct combinations of two anchors from given set of >= 2 anchors.
       distinct means that no combination of two anchors shall appear >1 in the output.
       e.g. (A1, A2) shall not appear again as (A2, A1)
    */
    static bool findAllAnchorCombinations(const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                          std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> & outputAnchorCombinations);

    static CircleIntersectionResult findTwoCirclesIntersections(const AnchorPositionTagDistance a1t,
                                                                const AnchorPositionTagDistance a2t,
                                                                LatLong & t, LatLong & t_prime);
    static void filterPositionCandidates(const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                         std::vector<LatLong> & positionCandidates,
                                         std::vector<LatLong> & filteredOut);
    static bool selectBestMatchingCandidate(const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                            const std::vector<LatLong> & positionCandidates,
                                            LatLong & bestMatchingCandidate);

    static double METER_TO_DEGREE(const double latitude);

public: // instance methods
    Location() {}
    Location(const Location&) = delete; // forbid copy constructor

    CalcResult calculatePosition(CalculationPhase & phase,
                                 const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                 LatLong & outputTagPosition, double & outputTagPositionErrorEstimate);

private: // static methods
    /* lat/long to ECEF */
    static void latlong2ecef(const LatLongAlt & latLong, ECEF & ecef);
    /* ECEF with reference lat/long to ENU */
    static void ecef2enu(const ECEF & ecef, const LatLongAlt & refLatLong, ENU & enu);
    /* ECEF to lat/long */
    static bool ecef2latLong(const ECEF & ecef, LatLongAlt & latLong);
    /* 3x3 rotation matrix */
    static void rotate(const double angle, const Axis axis, double R[3][3]);
    /* 3D rotation matrix to/from ECEF/ENU using reference lat/long */
    static void rotate3D(const LatLongAlt & latLong, double R[3][3]);
    /* Multiply 3x3 matrix times another 3x3 matrix C=AB */
    static void matrixMultiply3x3with3x3(const double A[3][3], const double B[3][3], double C[3][3]);
    /* Multiply 3x3 matrix times a 3x1 vector c=Ab */
    static void matrixMultiply3x3with3x1(const double A[3][3], const double b[3], double c[3]);
    /* Transpose a 3x3 matrix At = A' */
    static void transposeMatrix3x3(const double A[3][3], double At[3][3]);

private: // attributes
    static const char* TAG;

    std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> pairOfTwoAnchorsAndTheirDistanceToTag;
    std::vector<LatLong> positionCandidates;
};

}  // namespace uwb
}  // namespace esphome

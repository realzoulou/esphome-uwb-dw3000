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

    static CalcResult calculatePosition(const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                        LatLong & outputTagPosition, double & outputTagPositionErrorEstimate);

    /* get squerical distance between 2 points on earth in [m] using Haversine formula. */
    static double getHaversineDistance(const LatLong & from, const LatLong & to);

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

private:
    static const char* TAG;

    std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> pairOfTwoAnchorsAndTheirDistanceToTag;
    std::vector<LatLong> positionCandidates;
};

}  // namespace uwb
}  // namespace esphome

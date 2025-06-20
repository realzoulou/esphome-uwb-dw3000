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

#include "Location.h"

#include <iomanip>
#include <iostream>
#include <limits>
#include <math.h>

#ifdef ESP32
#include "esphome/core/log.h"
    #define LOC_LOGW(ostringstream) ESP_LOGW(TAG, "%s", ostringstream.str().c_str());
    #define LOC_LOGI(ostringstream) ESP_LOGI(TAG, "%s", ostringstream.str().c_str());
#else
    #define LOC_LOGW(ostringstream) std::clog << ostringstream.str() << std::endl;
    #define LOC_LOGI(ostringstream) std::cout << ostringstream.str() << std::endl;
#endif

namespace esphome {
namespace uwb {

#ifndef M_PI
#define M_PI            (3.14159265358979323846)
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD      ((double)(M_PI / 180.0))
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG      ((double)(180.0 / M_PI))
#endif

// https://en.wikipedia.org/wiki/Earth_ellipsoid (using WGS-84)
#ifndef EARTH_RADIUS_EQUATOR_METERS
#define EARTH_RADIUS_EQUATOR_METERS     ((double)(6378137.0))    // Earth radius at equator [m]
#endif
#ifndef EARTH_RADIUS_POLES_METERS
#define EARTH_RADIUS_POLES_METERS       ((double)(6356752.3142)) // Earth radius at the poles [m]
#endif
#ifndef EARTH_RADIUS_AVG_METERS
#define EARTH_RADIUS_AVG_METERS         ((double)(6371000.0))      // average earth radius in [km]
#endif
#ifndef EARTH_FLATTENING
#define EARTH_FLATTENING      ((double)(1.0 / 298.257223563)) // earth flattening
#endif
#ifndef NAV_E2
#define NAV_E2  ((2.0 - EARTH_FLATTENING) * EARTH_FLATTENING)
#endif

const char* Location::TAG = "location";

#define LOG_DEGREE_PRECISION  (7) // digits after decimal point
#define LOG_METER_PRECISION   (2) // digits after decimal point, ie. up to [cm]
#define LOG_XYZ_PRECISION    (10) // digits after decimal point for ENU/ECEF

void Location::LOG_ANCHOR_TO_STREAM(std::ostringstream & ostream, const AnchorPositionTagDistance & anchor) {
    ostream << std::hex << +anchor.anchorId << std::dec << "(";
    ostream << std::fixed << std::setprecision(LOG_DEGREE_PRECISION);
    ostream << +anchor.anchorPosition.latitude << "," << +anchor.anchorPosition.longitude << ",";
    ostream << std::fixed << std::setprecision(LOG_METER_PRECISION);
    ostream << +anchor.tagDistance << "m)";
}

void Location::LOG_ANCHORS_TO_STREAM(std::ostringstream & ostream, const std::vector<AnchorPositionTagDistance> & anchors) {
    ostream << "[";
    for (const AnchorPositionTagDistance & anchor : anchors) {
        ostream << "{";
        LOG_ANCHOR_TO_STREAM(ostream, anchor);
        ostream << "}";
    }
    ostream << "]";
}

double Location::METER_TO_DEGREE(const double latitude) {
    // What is 1 meter on earth at a certain latitude [°] ?
    /* 1. Earth radius at latitude
       https://rechneronline.de/earth-radius/
       latitude B, radius R, radius at equator r1, radius at pole r2
       R = √ [ (r1² * cos(B))² + (r2² * sin(B))² ] / [ (r1 * cos(B))² + (r2 * sin(B))² ]
    */
    const double r1 = EARTH_RADIUS_EQUATOR_METERS;
    const double r2 = EARTH_RADIUS_POLES_METERS;
    const double x = latitude * DEG_TO_RAD;
    const double r1sCos = r1*r1*std::cos(x); // r1² * cos(B)
    const double r1Cos  = r1*std::cos(x);    // r1 * cos(B)
    const double r2sSin = r2*r2*std::sin(x); // r2² * sin(B)
    const double r2Sin  = r2*std::sin(x);    // r2 * sin(B)
    const double R = std::sqrt( (r1sCos*r1sCos + r2sSin*r2sSin) / ( r1Cos*r1Cos  +  r2Sin*r2Sin) );
    /* 2. Circumference U of Earth at latitude: C = 2 * π * R */
    const double U = R * 2 * M_PI;
    /* 3. 1° is 360/U */
    return 360.0 / U;
}

CalcResult Location::calculatePosition(const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                       LatLong & outputTagPosition, double & outputTagPositionErrorEstimate) {
    Location l;
    CalculationPhase phase = CALC_RUN_ALL_PHASES_AT_ONCE;
    return l.calculatePosition(phase, inputAnchorPositionAndTagDistances, outputTagPosition, outputTagPositionErrorEstimate);
}

CalcResult Location::calculatePosition(CalculationPhase & phase,
                                       const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                       LatLong & outputTagPosition, double & outputTagPositionErrorEstimate) {
    bool ok;
    if ((phase & CALC_PHASE_INIT) == CALC_PHASE_INIT) {
        pairOfTwoAnchorsAndTheirDistanceToTag.clear();
        positionCandidates.clear();
        phase &= ~CALC_PHASE_INIT; // remove CALC_PHASE_INIT
    }

    /* Find all distinct combinations of two inputAnchorLocations. */
    if ((phase & CALC_PHASE_DONE_ANCHOR_COMBINATIONS) != CALC_PHASE_DONE_ANCHOR_COMBINATIONS) {
        ok = findAllAnchorCombinations(inputAnchorPositionAndTagDistances, pairOfTwoAnchorsAndTheirDistanceToTag);
        if (!ok) {
            return CALC_F_ANCHOR_COMBINATIONS;
        } else if ((phase & CALC_RUN_ALL_PHASES_AT_ONCE) != CALC_RUN_ALL_PHASES_AT_ONCE) {
            phase += CALC_PHASE_DONE_ANCHOR_COMBINATIONS;
            return CALC_PHASE_OK;
        }
    }
    /* For each pair of anchors with their tag distance, find the intersection positions of 2 circles.
       Each circle's center is the anchor position and circle radius is the distance to tag.
       The usually 2 positions are 'candidates' of the tag position relative to this pair of anchors.
    */
    if ((phase & CALC_PHASE_DONE_COLLECT_CANDIDATES) != CALC_PHASE_DONE_COLLECT_CANDIDATES) {
        for (const auto & p : pairOfTwoAnchorsAndTheirDistanceToTag) {
            LatLong t, t_prime;
            CircleIntersectionResult rc = findTwoCirclesIntersections(p.first, p.second, t, t_prime);
            if (CIRCLE_INTERSECT_OK ==  rc) {
                positionCandidates.push_back(t);
                if ((t != t_prime)) {
                    positionCandidates.push_back(t_prime);
                }
            } else if (CIRCLE_INTERSECT_ERROR_NO_INTERSECTION == rc) {
                // measured distances result in no circle intersections
                // increase both distances step-wise by 10cm, but max 1m
                AnchorPositionTagDistance modP1 = p.first, modP2 = p.second;
                double d; // distance increase [m] applied to both
                for (d = 0.1; d <= 1/*m*/; d += 0.1) {
                    modP1.tagDistance = p.first.tagDistance + d;
                    modP1.tagDistanceErrEstimate = p.first.tagDistanceErrEstimate + d;
                    modP2.tagDistance = p.second.tagDistance + d;
                    modP2.tagDistanceErrEstimate = p.second.tagDistanceErrEstimate + d;
                    rc = findTwoCirclesIntersections(modP1, modP2, t, t_prime);
                    if (CIRCLE_INTERSECT_ERROR_NO_INTERSECTION == rc) {
                        continue; // increase distances even more
                    } else if (CIRCLE_INTERSECT_OK == rc) {
                        // finally found intersections
                        positionCandidates.push_back(t);
                        if ((t != t_prime)) {
                            positionCandidates.push_back(t_prime);
                        }
                        std::ostringstream msg;
                        msg << "circle intersections found with each dist +" << +d << "m ";
                        LOG_ANCHOR_TO_STREAM(msg, p.first);
                        msg << " ";
                        LOG_ANCHOR_TO_STREAM(msg, p.second);
                        LOC_LOGW(msg);
                        break;
                    }
                }
                if (CIRCLE_INTERSECT_ERROR_NO_INTERSECTION == rc) {
                    // even with increasing distances, still no intersections
                    std::ostringstream msg;
                    msg << toString(rc) << ": ";
                    LOG_ANCHOR_TO_STREAM(msg, p.first);
                    msg << " ";
                    LOG_ANCHOR_TO_STREAM(msg, p.second);
                    msg << " even with each dist +" << +(d-0.1) << "m";
                    LOC_LOGW(msg);
                }
            } else if (CIRCLE_INTERSECT_ERROR_CONTAINED == rc) {
                /* no solution. one circle is contained in the other, or circles exactly equal. */
                std::ostringstream msg;
                msg << toString(rc) << ": ";
                LOG_ANCHOR_TO_STREAM(msg, p.first);
                msg << " ";
                LOG_ANCHOR_TO_STREAM(msg, p.second);
                LOC_LOGW(msg);
            }
        }
        // need at least 1 candidate
        if (positionCandidates.empty()) {
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_WARN
        {
            std::ostringstream msg;
            msg << "no position candidates found from anchor pairs:";
            LOC_LOGW(msg);
            unsigned cnt = 0;
            for (const auto & p : pairOfTwoAnchorsAndTheirDistanceToTag) {
                cnt++;
                msg = std::ostringstream();
                msg << " pair " << +cnt << ": ";
                LOG_ANCHOR_TO_STREAM(msg, p.first);
                msg << " ";
                LOG_ANCHOR_TO_STREAM(msg, p.second);
                LOC_LOGW(msg);
            }
        }
#endif // ESPHOME_LOG_LEVEL_WARN
            return CALC_F_NO_CANDIDATES;
        }
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_INFO
        {
            std::ostringstream msg;
            msg << +positionCandidates.size() <<  " position candidates:";
            LOC_LOGI(msg);
            unsigned cnt = 0;
            for (const auto & p : positionCandidates) {
                cnt++;
                msg = std::ostringstream();
                msg << std::fixed << std::setprecision(LOG_DEGREE_PRECISION);
                msg << " " << +cnt << ": " << +p.latitude << "," << +p.longitude;
                LOC_LOGI(msg);
            }
        }
#endif // ESPHOME_LOG_LEVEL_INFO
        if ((phase & CALC_RUN_ALL_PHASES_AT_ONCE) != CALC_RUN_ALL_PHASES_AT_ONCE) {
            phase += CALC_PHASE_DONE_COLLECT_CANDIDATES;
            return CALC_PHASE_OK;
        }
    }
    if ((phase & CALC_PHASE_DONE_FILTER_CANDIDATES) != CALC_PHASE_DONE_FILTER_CANDIDATES) {
        /* Filter position candidates that are not within the anchors area. */
        std::vector<LatLong> filteredOut;
        filterPositionCandidates(inputAnchorPositionAndTagDistances, positionCandidates, filteredOut);
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_INFO
        if (!filteredOut.empty()) {
            std::ostringstream msg;
            msg << +filteredOut.size() <<  " filtered out position candidates:";
            LOC_LOGI(msg);
            unsigned cnt = 0;
            for (const auto & p : filteredOut) {
                cnt++;
                msg = std::ostringstream();
                msg << " " << +cnt << ": " << std::fixed << std::setprecision(LOG_DEGREE_PRECISION)
                    << +p.latitude << "," << +p.longitude;
                LOC_LOGI(msg);
            }
        }
#endif // ESPHOME_LOG_LEVEL_INFO
        if ((phase & CALC_RUN_ALL_PHASES_AT_ONCE) != CALC_RUN_ALL_PHASES_AT_ONCE) {
            phase += CALC_PHASE_DONE_FILTER_CANDIDATES;
            return CALC_PHASE_OK;
        }
    }

    if ((phase & CALC_PHASE_DONE_SELECT_BEST_CANDIDATE) != CALC_PHASE_DONE_SELECT_BEST_CANDIDATE) {
        LatLong bestMatchingCandidate = {NAN, NAN};
        ok = selectBestMatchingCandidate(inputAnchorPositionAndTagDistances, positionCandidates, bestMatchingCandidate);
        if (!ok) {
            std::ostringstream msg;
            msg << "failed finding best matching position candidate";
            LOC_LOGW(msg);
            return CALC_F_BEST_MATCH;
        }
        if ((phase & CALC_RUN_ALL_PHASES_AT_ONCE) != CALC_RUN_ALL_PHASES_AT_ONCE) {
            phase += CALC_PHASE_DONE_SELECT_BEST_CANDIDATE;
        }
        outputTagPosition.latitude = bestMatchingCandidate.latitude;
        outputTagPosition.longitude = bestMatchingCandidate.longitude;
        // position error estimate is the square root of the sum of all anchor distance error estimates
        double errEstimate = 0;
        for (auto & a : inputAnchorPositionAndTagDistances) {
            if (!std::isnan(a.tagDistanceErrEstimate)) {
                errEstimate += a.tagDistanceErrEstimate;
            }
        }
        outputTagPositionErrorEstimate = std::sqrt(errEstimate);

#ifdef __UT_TEST__ // extra log in unit tests
        std::cout << std::fixed << std::setprecision(LOG_DEGREE_PRECISION)
                << "outputTagPosition (lat/lng)=" << +outputTagPosition.latitude << "/" << +outputTagPosition.longitude
                << std::setprecision(LOG_METER_PRECISION)
                << " errEst:" << +outputTagPositionErrorEstimate << "m"
                << std::endl;
#endif
    }
    return CALC_OK;
}

bool Location::isValid(const AnchorPositionTagDistance & a) {
    return (isValid(a.anchorPosition) && isDistancePlausible(a.tagDistance));
}

bool Location::isValid(const LatLong & a) {
    return (   (a.latitude > -90.0 && a.latitude < 90.0)
            && (a.longitude > -180.0 && a.longitude < 180.0)
            && (a.latitude != 0.0 && a.longitude != 0.0 )
            && (a.latitude != NAN && a.longitude != NAN )
           );
}
double Location::getHaversineDistance(const LatLong & from, const LatLong & to) {
    /* Haversine formula
       https://github.com/chrisveness/geodesy/blob/master/latlon-spherical.js#L189
       a = sin²(Δφ/2) + cos(φ1)⋅cos(φ2)⋅sin²(Δλ/2)
       δ = 2·atan2(√(a), √(1−a))
       see mathforum.org/library/drmath/view/51879.html for derivation (link dead?)
    */
    const double TO_RAD = M_PI / 180;
    const double R = EARTH_RADIUS_AVG_METERS;
    const double phi1 = from.latitude * DEG_TO_RAD, lamda1 = from.longitude * DEG_TO_RAD;
    const double phi2 = to.latitude * TO_RAD, lamda2 = to.longitude * TO_RAD;
    const double delta_phi = phi2 - phi1;
    const double delta_phi_half = delta_phi / 2;
    const double delta_lamda_half = (lamda2 - lamda1) / 2;

    const double a = std::sin(delta_phi_half)*std::sin(delta_phi_half) + std::cos(phi1) *std::cos(phi2) * std::sin(delta_lamda_half)*std::sin(delta_lamda_half);
    const double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));

    return R * c;
}

void Location::latLongToEnu(const LatLongAlt & latLong, const LatLongAlt & refLatLong, ENU & enu) {
    ECEF ecef;
    latlong2ecef(latLong, ecef);
    ecef2enu(ecef, refLatLong, enu);

    std::ostringstream info1;
    info1 << std::fixed << std::setprecision(LOG_DEGREE_PRECISION)
          << "latLongToEnu (" << +latLong.latitude << "/" << latLong.longitude
          << ", ref " << +refLatLong.latitude << "/" << +refLatLong.longitude
          << std::setprecision(LOG_XYZ_PRECISION)
          << ") = " << +enu.x << "," << +enu.y << "," << +enu.z;
    LOC_LOGI(info1);
}

bool Location::enuToLatLong(const ENU & enu, const LatLongAlt & refLatLong, LatLongAlt & latLong) {
    const double enu_xyz[3] = {enu.x, enu.y, enu.z};
    double diff_xyz[3], R[3][3], Rt[3][3];
    ECEF ecef, refEcef;

    latlong2ecef(refLatLong, refEcef);
    rotate3D(refLatLong, R);
    transposeMatrix3x3(R, Rt);
    matrixMultiply3x3with3x1(Rt, enu_xyz, diff_xyz);
    ecef.x = diff_xyz[0] + refEcef.x;
    ecef.y = diff_xyz[1] + refEcef.y;
    ecef.z = diff_xyz[2] + refEcef.z;
    const bool ok = ecef2latLong(ecef, latLong);
#ifdef __UT_TEST__ // extra log in unit tests
    std::ostringstream info1;
    info1 << std::fixed << std::setprecision(LOG_XYZ_PRECISION)
          << "enuToLatLong (" << +enu.x << "," << +enu.y << "," << +enu.z
          << std::setprecision(LOG_DEGREE_PRECISION)
          << ", ref " << +refLatLong.latitude << "/" << +refLatLong.longitude
          << ") = " << +latLong.latitude << "/" << +latLong.longitude
          << ": ok=" << +ok;
    LOC_LOGI(info1);
#endif
    return ok;
}

void Location::latlong2ecef(const LatLongAlt & latLong, ECEF & ecef) {
    double sLat = sin(latLong.latitude * DEG_TO_RAD);
    double cLat = cos(latLong.latitude * DEG_TO_RAD);
    double r_n = EARTH_RADIUS_EQUATOR_METERS / std::sqrt(1.0 - NAV_E2*sLat*sLat);
    ecef.x = (r_n + latLong.altitude) * cLat * std::cos(latLong.longitude*DEG_TO_RAD);
    ecef.y = (r_n + latLong.altitude) * cLat * std::sin(latLong.longitude*DEG_TO_RAD);
    ecef.z = (r_n * (1.0 - NAV_E2) + latLong.altitude) * sLat;
}

void Location::ecef2enu(const ECEF & ecef, const LatLongAlt & refLatLong, ENU & enu) {
    ECEF refEcef;
    latlong2ecef(refLatLong, refEcef);

    double diffEcef[3];
    diffEcef[0] = ecef.x - refEcef.x;
    diffEcef[1] = ecef.y - refEcef.y;
    diffEcef[2] = ecef.z - refEcef.z;

    double R[3][3];
    rotate3D(refLatLong, R);

    double R_enu[3];
    matrixMultiply3x3with3x1(R, diffEcef, R_enu);
    enu.x = R_enu[0];
    enu.y = R_enu[1];
    enu.z = R_enu[2];
}

void Location::rotate(const double angle, const Axis axis, double R[3][3]) {
    double cAng = std::cos(angle * DEG_TO_RAD);
    double sAng = std::sin(angle * DEG_TO_RAD);

    if (X_AXIS == axis) {
        R[0][0] = 1;
        R[0][1] = 0;
        R[0][2] = 0;
        R[1][0] = 0;
        R[2][0] = 0;
        R[1][1] = cAng;
        R[2][2] = cAng;
        R[1][2] = sAng;
        R[2][1] = -sAng;
    } else if (Y_AXIS == axis) {
        R[0][1] = 0;
        R[1][0] = 0;
        R[1][1] = 1;
        R[1][2] = 0;
        R[2][1] = 0;
        R[0][0] = cAng;
        R[2][2] = cAng;
        R[0][2] = -sAng;
        R[2][0] = sAng;
    } else if (Z_AXIS == axis) {
        R[2][0] = 0;
        R[2][1] = 0;
        R[2][2] = 1;
        R[0][2] = 0;
        R[1][2] = 0;
        R[0][0] = cAng;
        R[1][1] = cAng;
        R[1][0] = -sAng;
        R[0][1] = sAng;
    }
}

void Location::rotate3D(const LatLongAlt & latLong, double R[3][3]) {
    double R1[3][3], R2[3][3];

    rotate(90.0 + latLong.longitude, Z_AXIS, R1);
    rotate(90.0 - latLong.latitude, X_AXIS, R2);
    matrixMultiply3x3with3x3(R2, R1, R);
}

bool Location::ecef2latLong(const ECEF & ecef, LatLongAlt & latLong) {
    double rhoSquare = ecef.x*ecef.x + ecef.y*ecef.y;
    double rho = std::sqrt(rhoSquare);
    double tempLat = std::atan2(ecef.z, rho);
    double tempAlt = std::sqrt(rhoSquare + ecef.z*ecef.z) - EARTH_RADIUS_EQUATOR_METERS;
    double rhoError = 1000.0;
    double zError = 1000.0;

    int i = 0;
    while ((std::abs(rhoError) > 1e-6) || std::abs(zError) > 1e-6) {
        if (++i > 20) return false;

        double sLat = std::sin(tempLat);
        double cLat = std::cos(tempLat);
        double q = 1.0 - NAV_E2 * sLat*sLat;
        double r_n = EARTH_RADIUS_EQUATOR_METERS / std::sqrt(q);
        double drdl = r_n * NAV_E2 * sLat * cLat / q;
        rhoError = (r_n + tempAlt) * cLat - rho;
        zError = (r_n * (1.0 - NAV_E2) + tempAlt) * sLat - ecef.z;
        double aa = drdl * cLat - (r_n + tempAlt) * sLat;
        double bb = cLat;
        double cc = (1.0 - NAV_E2) * (drdl * sLat + r_n * cLat);
        double dd = sLat;
        double invdet = 1.0 / (aa * dd - bb * cc);
        tempLat = tempLat - invdet * ( dd * rhoError - bb * zError);
        tempAlt = tempAlt - invdet * (-cc * rhoError + aa * zError);
    }
    latLong.latitude = tempLat * RAD_TO_DEG;
    latLong.longitude = std::atan2(ecef.y, ecef.x) * RAD_TO_DEG;
    latLong.altitude = tempAlt;
    return true;
}

void Location::matrixMultiply3x3with3x3(const double A[3][3], const double B[3][3], double C[3][3]) {
    C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
    C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1];
    C[0][2] = A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2];
    C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
    C[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1];
    C[1][2] = A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2];
    C[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0];
    C[2][1] = A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1];
    C[2][2] = A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2];
}

void Location::matrixMultiply3x3with3x1(const double A[3][3], const double b[3], double c[3]) {
    c[0] = A[0][0] * b[0] + A[0][1] * b[1] + A[0][2] * b[2];
    c[1] = A[1][0] * b[0] + A[1][1] * b[1] + A[1][2] * b[2];
    c[2] = A[2][0] * b[0] + A[2][1] * b[1] + A[2][2] * b[2];
}

void Location::transposeMatrix3x3(const double A[3][3], double At[3][3]) {
    At[0][0] = A[0][0];
    At[0][1] = A[1][0];
    At[0][2] = A[2][0];
    At[1][0] = A[0][1];
    At[1][1] = A[1][1];
    At[1][2] = A[2][1];
    At[2][0] = A[0][2];
    At[2][1] = A[1][2];
    At[2][2] = A[2][2];
}

bool Location::findAllAnchorCombinations(const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                          std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> & outputAnchorCombinations) {
    const std::size_t inputAnchorsNum = inputAnchorPositionAndTagDistances.size();
    if (inputAnchorsNum < 2) {
        std::ostringstream msg;
        msg << "findAllAnchorCombinations: inputAnchorsNum " << +inputAnchorsNum << " <2";
        LOC_LOGW(msg);
        return false;
    }

    for (std::size_t i = 0; i < inputAnchorsNum - 1; i++) {
        for (std::size_t j = i+1; j < inputAnchorsNum; j++) {
            outputAnchorCombinations.push_back(std::make_pair(inputAnchorPositionAndTagDistances[i], inputAnchorPositionAndTagDistances[j]));
        }
    }
    return true;
}

void Location::filterPositionCandidates(const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                        std::vector<LatLong> & positionCandidates, std::vector<LatLong> & filteredOut) {
    // ensure nothing else is in removed vector from caller than why we put into below
    filteredOut.clear();

    // need min. 3 anchors and min. 1 position candidate
    // with 2 anchors there would be no 'area', but a line
    if ((inputAnchorPositionAndTagDistances.size() < 3) || positionCandidates.empty())
        return;

    // find minimum and maximum of X,Y coordinates of anchors
    double minX = std::numeric_limits<double>::max(),
           minY = std::numeric_limits<double>::max(),
           maxX = std::numeric_limits<double>::lowest(),
           maxY = std::numeric_limits<double>::lowest();
    for (const AnchorPositionTagDistance & a : inputAnchorPositionAndTagDistances) {
        if (std::isnan(a.anchorPosition.latitude) || std::isnan(a.anchorPosition.longitude)) {
            std::ostringstream msg;
            LOG_ANCHOR_TO_STREAM(msg, a);
            LOC_LOGW(msg);
            continue; // skip anchor with nan X or Y coordinate
        }
        minX = std::min(minX, a.anchorPosition.longitude);
        maxX = std::max(maxX, a.anchorPosition.longitude);
        minY = std::min(minY, a.anchorPosition.latitude);
        maxY = std::max(maxY, a.anchorPosition.latitude);
    }
    // collect references of positionCandidates array to stay in
    std::vector<LatLong> filteredPositionCandidates;
    // collect references of positionCandidates array to be removed
    std::vector<LatLong> toBeRemoved;
    // filter out candidates that are outside of minimum and maximum of X,Y coordinates of anchors
    for (LatLong & p : positionCandidates) {
        if ((p.latitude > maxY) || (p.latitude < minY) || (p.longitude > maxX) || (p.longitude < minX)
            || std::isnan(p.latitude) || std::isnan(p.longitude)) {
            toBeRemoved.push_back(p);
        } else {
            filteredPositionCandidates.push_back(p);
        }
    }
    if (toBeRemoved.empty()) {
        return; // nothing filtered
    }
    // remove all old position candidates
    positionCandidates.clear();
    // add back the filtered ones
    positionCandidates.assign(filteredPositionCandidates.begin(), filteredPositionCandidates.end());
    // fill filteredOut vector
    filteredOut.assign(toBeRemoved.begin(), toBeRemoved.end());
}

bool Location::selectBestMatchingCandidate(const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                           const std::vector<LatLong> & positionCandidates,
                                           LatLong & bestMatchingCandidate) {
    // need at least 2 anchors and at least 1 candidate
    if (inputAnchorPositionAndTagDistances.size() < 2 || positionCandidates.empty())
        return false;
    // if only 1 candidate, this is the best we have
    if (positionCandidates.size() == 1) {
        if (std::isnan(positionCandidates[0].latitude) || std::isnan(positionCandidates[0].longitude)) {
            // NAN is not a position candidate
            return false;
        }
        bestMatchingCandidate.latitude = positionCandidates[0].latitude;
        bestMatchingCandidate.longitude = positionCandidates[0].longitude;
        return true;
    }
    LatLong bestCandidateSoFar = {NAN, NAN};
    double distMin = std::numeric_limits<double>::max();
    // find the position candidate with the least distances to anchors
    for (const LatLong & c : positionCandidates) {
        double distSum = 0;
        for (const AnchorPositionTagDistance & a : inputAnchorPositionAndTagDistances) {
            distSum += getHaversineDistance(c, a.anchorPosition);
        }
        if (distSum < distMin) {
            bestCandidateSoFar = c;
            distMin = distSum;
        }
    }
    if (std::isnan(bestCandidateSoFar.latitude) || std::isnan(bestCandidateSoFar.longitude)) {
        return false; // search above failed?, should be impossible
    }
    bestMatchingCandidate.latitude = bestCandidateSoFar.latitude;
    bestMatchingCandidate.longitude = bestCandidateSoFar.longitude;
    return true;
}

CircleIntersectionResult Location::findTwoCirclesIntersections(const AnchorPositionTagDistance a1t,
                                                               const AnchorPositionTagDistance a2t,
                                                               LatLong & t, LatLong & t_prime) {
    /* First, convert geodesic WGS-84 to cartesian coordinate system. */
    /* a1t is our reference point*/
    const LatLongAlt refA1 = { a1t.anchorPosition.latitude, a1t.anchorPosition.longitude, 0.0};
    const LatLongAlt lla2 = { a2t.anchorPosition.latitude, a2t.anchorPosition.longitude, 0.0};
    ENU enuA2;
    /* convert a2t to ENU relative to a1t */
    latLongToEnu(lla2, refA1, enuA2);
    /* a1t is our reference point, hence x0/y0 are 0.0 */
    const double x0 = 0.0, y0 = 0.0, r0 = a1t.tagDistance;
    /* x1/y1/r1 are now in cartesian coordinate system [m] */
    const double x1 = enuA2.x, y1 = enuA2.y, r1 = a2t.tagDistance;
    double x, y, x_prime, y_prime;

    if (std::isnan(x0) || std::isnan(y0) || std::isnan(r0) ||
        std::isnan(x1) || std::isnan(y1) || std::isnan(r1) ||
        r0 < 0 || r1 < 0) {
        return CIRCLE_INTERSECT_ERROR_INPUT;
    }

    /* ported from https://paulbourke.net/geometry/circlesphere/tvoght.c */
    double a, dx, dy, d, h, rx, ry;
    double x2, y2;

    /* dx and dy are the vertical and horizontal distances between
    * the circle centers.
    */
    dx = x1 - x0;
    dy = y1 - y0;

    /* Determine the straight-line distance between the centers. */
    //d = std::sqrt((dy*dy) + (dx*dx));
    d = std::hypot(dx,dy); // Suggested by Keith Briggs

    /* Check for solvability. */
    if (d > (r0 + r1))
    {
        return CIRCLE_INTERSECT_ERROR_NO_INTERSECTION;
    }
    if (d <= std::fabs(r0 - r1)) // was originally: if (d < fabs(r0 - r1))
    {
        return CIRCLE_INTERSECT_ERROR_CONTAINED;
    }

    /* 'point 2' is the point where the line through the circle
    * intersection points crosses the line between the circle centers.
    */

    /* Determine the distance from point 0 to point 2. */
    a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;

    /* Determine the coordinates of point 2. */
    x2 = x0 + (dx * a/d);
    y2 = y0 + (dy * a/d);

    /* Determine the distance from point 2 to either of the
    * intersection points.
    */
    h = std::sqrt((r0*r0) - (a*a));

    /* Now determine the offsets of the intersection points from
    * point 2.
    */
    rx = -dy * (h/d);
    ry = dx * (h/d);

    /* Determine the absolute intersection points. */
    x = x2 + rx;
    x_prime = x2 - rx;
    y = y2 + ry;
    y_prime = y2 - ry;

    /* end of: ported from https://paulbourke.net/geometry/circlesphere/tvoght.c */

    /* Finally, convert back from cartesian to geodesic WGS-84 coordinate system. */
    const ENU enuT = {x, y, 0.0}, enuT_prime = {x_prime, y_prime, 0.0};
    LatLongAlt llaT, llaT_prime;
    const bool okT = enuToLatLong(enuT, refA1, llaT);
    const bool okT_prime = enuToLatLong(enuT_prime, refA1, llaT_prime);
    if (!okT && !okT_prime) {
        return CIRCLE_INTERSECT_ERROR_NO_INTERSECTION;
    }
    if (okT) {
        t.latitude = llaT.latitude;
        t.longitude = llaT.longitude;
    } else {
        t.latitude = NAN;
        t.longitude = NAN;
    }
    if (okT_prime) {
        t_prime.latitude = llaT_prime.latitude;
        t_prime.longitude = llaT_prime.longitude;
    } else {
        t_prime.latitude = NAN;
        t_prime.longitude = NAN;
    }
    return CIRCLE_INTERSECT_OK;
}

}  // namespace uwb
}  // namespace esphome

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
    #define LOC_LOGW(ostringstream) std::clog << msg.str() << std::endl;
    #define LOC_LOGI(ostringstream) std::cout << msg.str() << std::endl;
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

const char* Location::TAG = "location";

#define LOG_DEGREE_PRECISION  (7) // digits after decimal point
#define LOG_METER_PRECISION   (2) // digits after decimal point, ie. up to [cm]

void Location::LOG_ANCHOR_TO_STREAM(std::ostringstream & ostream, const AnchorPositionTagDistance & anchor) {
    ostream << std::hex << +anchor.anchorId << std::dec << "(";
    ostream << std::fixed << std::setprecision(LOG_DEGREE_PRECISION);
    ostream << +anchor.anchorPosition.latitude << "," << +anchor.anchorPosition.longitude << ",";
    ostream << std::fixed << std::setprecision(LOG_METER_PRECISION);
    ostream << +anchor.tagDistance << "m) ";
}

double Location::METER_TO_DEGREE(const double latitude) {
    // What is 1 meter on earth at a certain latitude [°] ?
    /* 1. Earth radius at latitude
       https://rechneronline.de/earth-radius/
       latitude B, radius R, radius at equator r1, radius at pole r2
       R = √ [ (r1² * cos(B))² + (r2² * sin(B))² ] / [ (r1 * cos(B))² + (r2 * sin(B))² ]
    */
    const double r1 = 6378137;   // Earth radius at equator [m]
    const double r2 = 6356752.3; // Earth radius at the poles [m]
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

    /* Find all distinct combinations of two inputAnchorLocations. */
    std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> pairOfTwoAnchorsAndTheirDistanceToTag;
    bool ok = findAllAnchorCombinations(inputAnchorPositionAndTagDistances, pairOfTwoAnchorsAndTheirDistanceToTag);
    if (!ok) return CALC_F_ANCHOR_COMBINATIONS;

    /* For each pair of anchors with their tag distance, find the intersection positions of 2 circles.
       Each circle's center is the anchor position and circle radius is the distance to tag.
       The usually 2 positions are 'candidates' of the tag position relative to this pair of anchors.
    */
    std::vector<LatLong> positionCandidates;
    for (const auto & p : pairOfTwoAnchorsAndTheirDistanceToTag) {
        LatLong t, t_prime;
        if (CIRCLE_INTERSECT_OK == findTwoCirclesIntersections(p.first, p.second, t, t_prime)) {
            positionCandidates.push_back(t);
            if ((t != t_prime)) {
                positionCandidates.push_back(t_prime);
            }
        }
    }
    // need at least 1 candidate
    if (positionCandidates.empty()) {
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_WARN
        std::ostringstream msg;
        msg << "no position candidates found from anchor pairs:";
        LOC_LOGW(msg);
        unsigned cnt = 0;
        for (const auto & p : pairOfTwoAnchorsAndTheirDistanceToTag) {
            cnt++;
            msg = std::ostringstream();
            msg << " pair " << +cnt << ": ";
            LOG_ANCHOR_TO_STREAM(msg, p.first);
            LOG_ANCHOR_TO_STREAM(msg, p.second);
            LOC_LOGW(msg);
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
    LatLong bestMatchingCandidate = {NAN, NAN};
    ok = selectBestMatchingCandidate(inputAnchorPositionAndTagDistances, positionCandidates, bestMatchingCandidate);
    if (!ok) {
        std::ostringstream msg;
        msg << "failed to find a best matching position candidate";
        LOC_LOGW(msg);
        return CALC_F_BEST_MATCH;
    }
    outputTagPosition.latitude = bestMatchingCandidate.latitude;
    outputTagPosition.longitude = bestMatchingCandidate.longitude;
    outputTagPositionErrorEstimate = 0; // if distances would be accurate then the resulting position has no error
#ifdef __UT_TEST__ // extra log in unit tests
    std::cout << "outputTagPosition (lat/lng)=" << +outputTagPosition.latitude << "/" << +outputTagPosition.longitude
              << " errEst:" << +outputTagPositionErrorEstimate << "m"
              << std::endl;
#endif
    return CALC_OK;
}

bool Location::isValid(const AnchorPositionTagDistance & a) {
    return (   isValid(a.anchorPosition)
            && (a.tagDistance > 0.0)
            && (a.tagDistance < UWB_MAX_REACH_METER)
           );
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
    const double R = 6371e3; // average earth radius in [km]
    const double phi1 = from.latitude * DEG_TO_RAD, lamda1 = from.longitude * DEG_TO_RAD;
    const double phi2 = to.latitude * TO_RAD, lamda2 = to.longitude * TO_RAD;
    const double delta_phi = phi2 - phi1;
    const double delta_phi_half = delta_phi / 2;
    const double delta_lamda_half = (lamda2 - lamda1) / 2;

    const double a = std::sin(delta_phi_half)*std::sin(delta_phi_half) + std::cos(phi1) *std::cos(phi2) * std::sin(delta_lamda_half)*std::sin(delta_lamda_half);
    const double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));

    return R * c;
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
        if (std::isnan(bestMatchingCandidate.latitude) || std::isnan(bestMatchingCandidate.longitude)) {
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
    const double x0 = a1t.anchorPosition.longitude * DEG_TO_RAD,
                 y0 = a1t.anchorPosition.latitude * DEG_TO_RAD,
                 r0 = a1t.tagDistance * METER_TO_DEGREE(a1t.anchorPosition.latitude) * DEG_TO_RAD;
    const double x1 = a2t.anchorPosition.longitude * DEG_TO_RAD,
                 y1 = a2t.anchorPosition.latitude * DEG_TO_RAD,
                 r1 = a2t.tagDistance * METER_TO_DEGREE(a2t.anchorPosition.latitude) * DEG_TO_RAD;
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
        /* no solution. circles do not intersect. */
        std::ostringstream msg;
        msg << "circles do not intersect: ";
        LOG_ANCHOR_TO_STREAM(msg, a1t);
        LOG_ANCHOR_TO_STREAM(msg, a2t);
        LOC_LOGW(msg);
        return CIRCLE_INTERSECT_ERROR_NO_INTERSECTION;
    }
    if (d <= std::fabs(r0 - r1)) // was originally: if (d < fabs(r0 - r1))
    {
        /* no solution. one circle is contained in the other, or circles exactly equal. */
        std::ostringstream msg;
        msg << "circles contained in each other: ";
        LOG_ANCHOR_TO_STREAM(msg, a1t);
        LOG_ANCHOR_TO_STREAM(msg, a2t);
        LOC_LOGW(msg);
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

    t.latitude =  y * RAD_TO_DEG;
    t.longitude = x * RAD_TO_DEG;
    t_prime.latitude = y_prime * RAD_TO_DEG;
    t_prime.longitude = x_prime * RAD_TO_DEG;
    return CIRCLE_INTERSECT_OK;
}

}  // namespace uwb
}  // namespace esphome
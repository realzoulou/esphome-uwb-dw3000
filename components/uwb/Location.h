#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>

#ifndef __UT_TEST__
#define UT_VISIBILITY_PRIVATE   private
#define UT_VISIBILITY_PROTECTED protected
#else
#define UT_VISIBILITY_PRIVATE   public
#define UT_VISIBILITY_PROTECTED public
#endif

namespace esphome {
namespace uwb {

typedef struct sLatLong {
    double latitude;  // degree
    double longitude; // degree
} LatLong;

inline bool operator==(const LatLong& lhs, const LatLong& rhs) {
    return lhs.latitude == rhs.latitude && lhs.longitude == rhs.longitude;
}
inline bool operator!=(const LatLong& lhs, const LatLong& rhs) {
    return !(lhs==rhs);
}
typedef struct sAnchorPositionTagDistance{
    LatLong anchorPosition;
    double tagDistance;
} AnchorPositionTagDistance;

inline bool operator==(const AnchorPositionTagDistance& lhs, const AnchorPositionTagDistance& rhs) {
    return lhs.anchorPosition == rhs.anchorPosition && lhs.tagDistance == rhs.tagDistance;
}
inline bool operator!=(const AnchorPositionTagDistance& lhs, const AnchorPositionTagDistance& rhs) {
    return !(lhs==rhs);
}

typedef struct sBoundingRect {
    /* LatLong of the most-western and most-southiest point of the rectangle. */
    LatLong westSouthiest;
    /* width = longitude [°]. */
    double width;
    /* height = latitude [°]. */
    double height;
} BoundingRect;

inline bool operator==(const BoundingRect& lhs, const BoundingRect& rhs) {
    return lhs.westSouthiest.latitude == rhs.westSouthiest.latitude
        && lhs.westSouthiest.longitude == rhs.westSouthiest.longitude
        &&  lhs.width == rhs.width && lhs.height == rhs.height;
}
inline bool operator!=(const BoundingRect& lhs, const BoundingRect& rhs) {
    return !(lhs==rhs);
}

class Location {

public:
    static bool calculatePosition(const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                  LatLong & outputTagPosition, double & outputTagPositionErrorEstimate);

UT_VISIBILITY_PRIVATE:
    /* get squerical distance between 2 points on earth in [m] */
    static double getDistance(const LatLong a, const LatLong b);

    /* find all distinct combinations of two anchors from given set of >= 2 anchors.
       distinct means that no combination of two anchors shall appear >1 in the output.
       e.g. (A1, A2) shall not appear again as (A2, A1)
    */
    static bool findAllAnchorCombinations(const std::vector<AnchorPositionTagDistance> & inputAnchorPositionAndTagDistances,
                                          std::vector<std::pair<AnchorPositionTagDistance, AnchorPositionTagDistance>> & outputAnchorCombinations);

    static bool findBoundingRectangle(const std::vector<LatLong> inputPositions, BoundingRect & outRect);
    static bool findTwoCirclesIntersections(const AnchorPositionTagDistance a1t,
                                            const AnchorPositionTagDistance a2t,
                                            LatLong & t, LatLong & t_prime);
    static double METER_TO_DEGREE(const double latitude);
};

}  // namespace uwb
}  // namespace esphome
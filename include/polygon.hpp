#ifndef _POLYGON_H_
#define _POLYGON_H_

#include <algorithm>
#include <limits>
#include <type_traits>

namespace geo {

template <typename T>
struct Point {
    T x;
    T y;

    constexpr Point(T x, T y) : x(x), y(y) {}

    inline bool operator==(const Point<T>& other) const { return x == other.x && y == other.y; }
};

template <typename T>
struct Line {
    const Point<T>& p1;
    const Point<T>& p2;

    constexpr Line(const Point<T>& p1, const Point<T>& p2) : p1(p1), p2(p2) {}

    inline bool operator==(const Line<T>& other) const { return p1 == other.p1 && p2 == other.p2; }

    enum direction_t { DIR_UP, DIR_DOWN, DIR_NONE };

    inline direction_t getDirection() const {
        if (p1.y < p2.y) {
            return DIR_UP;
        } else if (p1.y > p2.y) {
            return DIR_DOWN;
        } else {
            return DIR_NONE;
        }
    }
};

template <typename T>
class Polygon {
   public:
    // Array of points its length
    const Point<T>* const points;
    inline size_t getCount() const { return _count; }

    /**
     * @brief Bounding box of optimized check, good for multiple polygons
     *
     */
    class BoundingBox {
        friend Polygon;

       public:
        T xMin;
        T xMax;
        T yMin;
        T yMax;

        constexpr BoundingBox(const T xMin, const T xMax, const T yMin, const T yMax)
            : xMin(xMin), xMax(xMax), yMin(yMin), yMax(yMax) {}

        constexpr BoundingBox()
            : xMin(std::numeric_limits<T>::max()),
              xMax(std::numeric_limits<T>::min()),
              yMin(std::numeric_limits<T>::max()),
              yMax(std::numeric_limits<T>::min()) {}

        inline bool checkPointInBoundingBox(const Point<T>& point) const {
            if (point.x < xMin || point.x > xMax || point.y < yMin || point.y > yMax) {
                return false;
            } else {
                return true;
            }
        }

       private:
        inline void _update(const Point<T>& point) {
            // X Planes
            if (point.x < xMin) {
                xMin = point.x;
            }

            if (point.x > xMax) {
                xMax = point.x;
            }

            // Y Planes
            if (point.y < yMin) {
                yMin = point.y;
            }

            if (point.y > yMax) {
                yMax = point.y;
            }
        }

        inline void _update(const Point<T>* points, const size_t count) {
            for (size_t i = 0; i < count; ++i) {
                _update(points[i]);
            }
        }
    };

    BoundingBox boundingBox;

    /**
     * @brief Construct a new Polygon
     *
     * @param points
     * @param count
     */
    inline Polygon(const Point<T>* points, const size_t count) : points(points) {
        // Check if the point is of integer type
        // static_assert(std::is_integral<T>::value, "Only integer types are supported");

        // Check if last and first point are equal, if so remove the last point
        const size_t lastIndex = count - 1;
        if (points[0].x == points[lastIndex].x && points[0].y == points[lastIndex].y) {
            _count = lastIndex;
        } else {
            _count = count;
        }

        // Calc bounding box
        boundingBox._update(points, _count);
    }

    inline bool isValidPolygon() const {
        // Check if at least 3 points
        if (_count < 3) {
            return false;
        }

        // Check if at least 3 unique points. The loop below starts at the second point and only
        // ever compares backwards, so the first point is never tested and has to be counted here.
        size_t uniquePoints = 1;

        for (size_t i = 1; i < _count; ++i) {
            bool isUnique = true;

            // Compare to all previous points
            for (size_t j = 0; j < i; ++j) {
                if (points[i] == points[j]) {
                    isUnique = false;
                    break;
                }
            }

            // If unique, increment counter
            if (isUnique) {
                if (++uniquePoints >= 3) {
                    return true;
                }
            }
        }

        return false;
    }

    /**
     * @brief Check if a point is inside the defined polygon
     *
     * @param point
     * @return true
     * @return false
     */
    inline bool checkPointInPolygon(const Point<T>& point) const {
        if (!isValidPolygon()) return false;
        if (!boundingBox.checkPointInBoundingBox(point)) return false;

        // If the number of line crossings is odd, the point is inside the polygon.
        // Otherwise outside.
        // By simply toggling a boolean value every time a line is crossed, we can
        // determine if the point is inside the polygon or not.
        bool isInside = false;

        // Get direction of the line previous to the first line
        auto prevLineDirection = Line<T>::DIR_NONE;
        for (size_t i = _count - 1, j = _count - 2; i > 0; i = j--) {
            const Line<T> line(points[j], points[i]);
            prevLineDirection = line.getDirection();

            // If none, check next previous point
            if (prevLineDirection == Line<T>::DIR_NONE) continue;

            // Otherwise we have a direction -> break
            break;
        }

        // Check intersections between the ray and every side of the polygon.
        for (size_t i = 0, j = _count - 1; i < _count; j = i++) {
            const Line<T> line(points[j], points[i]);

            switch (_pointLineIntersects(point, line, prevLineDirection)) {
                case ON_LINE:
                    // On-Line is always inside
                    return true;

                case INTERSECT:
                    isInside = !isInside;
                    break;

                default:
                    break;
            }
        }

        return isInside;
    }

   private:
    size_t _count;

    /**
     * @brief Type used for the intermediate calculations in _pointLineIntersects().
     *
     * Both the difference of two coordinates and the product of two such differences have to be
     * representable, which needs more bits than T itself offers. The bounding box check in
     * checkPointInPolygon() limits every difference to the extent of the polygon, so the
     * calculation is exact as long as the bounding box' width multiplied by its height fits in
     * here.
     */
#if defined(__SIZEOF_INT128__) && !defined(POLYGON_NO_INT128)
    using intermediate_t = std::conditional_t<std::is_integral_v<T>, __int128, long double>;
#else
    // 32 bit targets have no __int128. long long still covers every polygon whose bounding box
    // area fits in 63 bits, which includes one spanning the whole globe at the 1e7 degree
    // precision this library is usually fed with: 180e7 * 360e7 = 6.5e18 against 9.2e18.
    // Define POLYGON_NO_INT128 to compile this variant on a 64 bit host, for testing.
    using intermediate_t = std::conditional_t<std::is_integral_v<T>, long long, long double>;
#endif

    enum line_intersect_t { INTERSECT, NO_INTERSECT, ON_LINE };

    /**
     * @brief Checks if a line starting from the point and going to the right (positive x-axis)
     * intersects with the given, fixed-length line. First point inclusive, last point exclusive.
     *
     * @param point
     * @param line
     * @param lineDirection Last line's line direction, will update with current line's direction
     * @return true
     * @return false
     */
    static inline line_intersect_t _pointLineIntersects(const Point<T>& point, const Line<T>& line,
                                                        Line<T>::direction_t& lineDirectionInOut) {
        /*
            Will take the point and draw a line to the right (bigger x-values).
            Then checks if this line intersects with the given line
        */

        // First, get line direction
        const auto lineDirectionCurrent = line.getDirection();

        // Special-Case: Horizontal line (Included line with two equal points)
        if (lineDirectionCurrent == Line<T>::DIR_NONE) {
            // -> Horizontal line -> Check if point is on the line
            if (point.y == line.p1.y && point.x >= std::min(line.p1.x, line.p2.x) &&
                point.x < std::max(line.p1.x, line.p2.x)) {
                return ON_LINE;
            }

            // Skip none-direction (horizontal lines)
            return NO_INTERSECT;
        }

        // Save last and current direction
        const auto lineDirectionLast = lineDirectionInOut;
        lineDirectionInOut = lineDirectionCurrent;

        // Check if point is exactly at first point of line
        if (point.y == line.p1.y) {
            // -> Just check if we are left of the point and if the direction did NOT change
            return (point.x <= line.p1.x && lineDirectionLast == lineDirectionCurrent)
                       ? INTERSECT
                       : NO_INTERSECT;
        }

        // Last point is exclusive, so we need to check if the point is at the same height as the
        // last point of the line. If so, always return NO_INTERSECT
        if (point.y == line.p2.y) {
            return NO_INTERSECT;
        }

        // Define upper and lower points
        const Point<T>& upperPoint = line.p1.y > line.p2.y ? line.p1 : line.p2;
        const Point<T>& lowerPoint = line.p1.y < line.p2.y ? line.p1 : line.p2;

        // Check if the point is between the line, otherwise it can't intersect at all
        // The first Point of the line is always included, the last point is exclusive
        if (point.y < lowerPoint.y || point.y > upperPoint.y || point.y == line.p2.y) {
            return NO_INTERSECT;
        }

        // Special-Case: Vertical line
        if (lowerPoint.x == upperPoint.x) {
            // Check if point is exactly on the line
            if (point.x == lowerPoint.x) {
                return ON_LINE;
            }

            // Point is on the height of line, but is it left of it too?
            if (point.x <= lowerPoint.x) {
                return INTERSECT;
            } else {
                return NO_INTERSECT;
            }
        }

        // Normalize points
        const intermediate_t lineX = (intermediate_t)upperPoint.x - (intermediate_t)lowerPoint.x;
        const intermediate_t lineY = (intermediate_t)upperPoint.y - (intermediate_t)lowerPoint.y;
        const intermediate_t pointX = (intermediate_t)point.x - (intermediate_t)lowerPoint.x;
        const intermediate_t pointY = (intermediate_t)point.y - (intermediate_t)lowerPoint.y;

        // Get the point on the line at height of the given point/line to the right and check
        // if the point is left of this point. If so, the horizontal line intersects the given line.
        //
        // This is the cross multiplied form of 'pointX <= (pointY * lineX) / lineY'. Multiplying
        // instead of dividing keeps the comparison exact, and both the differences above and the
        // products here overflow T for anything but very short lines, hence intermediate_t. lineY
        // is always > 0 here, so the inequality does not flip: upperPoint is the point with the
        // bigger y and the equal-y case was already handled as a horizontal line above.
        const bool pointLeftOfLine = pointX * lineY <= pointY * lineX;

        if (pointLeftOfLine) {
            // Check if point is at same X-Position as first point of line. If so,
            // dont count to not double count two lines if point is at the same height
            // as the line end & begin
            if (point.y == line.p1.y) {
                return NO_INTERSECT;
            } else {
                return INTERSECT;
            }
        } else {
            return NO_INTERSECT;
        }
    }
};

}   // namespace geo

#endif /* _POLYGON_H_ */

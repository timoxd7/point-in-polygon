/**
 * Self contained test for include/polygon.hpp. No dependencies, no test framework.
 *
 * Build and run:
 *   g++ -std=c++20 -I include test/test_polygon.cpp -o test_polygon && ./test_polygon
 *
 * Every check runs for several coordinate types. Coordinates are always generated as whole
 * numbers so that the same values can be fed to an exact __int128 reference implementation,
 * independent of the type the polygon itself is instantiated with.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "polygon.hpp"

// ---------------------------------------------------------------------------------------------
// Test harness
// ---------------------------------------------------------------------------------------------

static int g_failed = 0;
static int g_checks = 0;

static void check(bool condition, const std::string& what) {
    ++g_checks;

    if (!condition) {
        ++g_failed;
        std::printf("  FAIL  %s\n", what.c_str());
    }
}

// ---------------------------------------------------------------------------------------------
// Exact reference implementation
//
// Works on the whole numbered coordinates directly, in __int128, so it is exact for every
// coordinate range the tests use. Points exactly on the outline are reported separately,
// because polygon.hpp is free to call those either way.
// ---------------------------------------------------------------------------------------------

using wide_t = __int128;

// Mirrors the choice polygon.hpp makes for its own intermediate type. A 32 bit target has no
// __int128, so the tests have to keep the coordinate ranges inside what the narrower type can
// represent exactly, otherwise they would assert a guarantee the header never made.
#if defined(__SIZEOF_INT128__) && !defined(POLYGON_NO_INT128)
static constexpr int INTERMEDIATE_BITS = 128;
#else
static constexpr int INTERMEDIATE_BITS = 64;
#endif

/**
 * @brief Largest coordinate magnitude that keeps every product of two coordinate deltas exact.
 *
 * A delta spans at most 2 * extent, and two of them are multiplied, so 2 * extent has to fit in
 * half of the intermediate type's value bits.
 */
static int64_t exactExtent(int64_t typeLimit) {
    const int halfBits = (INTERMEDIATE_BITS - 2) / 2;
    if (halfBits >= 62) {
        return typeLimit;
    }

    const int64_t limit = ((int64_t)1 << halfBits) / 2;
    return typeLimit < limit ? typeLimit : limit;
}

struct RefPoint {
    wide_t x;
    wide_t y;
};

enum class Location { Outside, Inside, OnOutline };

static Location referenceLocation(const std::vector<RefPoint>& polygon, const RefPoint& point) {
    const size_t count = polygon.size();

    for (size_t i = 0, j = count - 1; i < count; j = i++) {
        const wide_t ax = polygon[j].x, ay = polygon[j].y;
        const wide_t bx = polygon[i].x, by = polygon[i].y;

        const wide_t cross = (bx - ax) * (point.y - ay) - (by - ay) * (point.x - ax);

        if (cross == 0 && point.x >= std::min(ax, bx) && point.x <= std::max(ax, bx) &&
            point.y >= std::min(ay, by) && point.y <= std::max(ay, by)) {
            return Location::OnOutline;
        }
    }

    bool inside = false;

    for (size_t i = 0, j = count - 1; i < count; j = i++) {
        const wide_t ax = polygon[j].x, ay = polygon[j].y;
        const wide_t bx = polygon[i].x, by = polygon[i].y;

        if ((ay > point.y) != (by > point.y)) {
            // point.x < ax + (bx - ax) * (point.y - ay) / (by - ay), cross multiplied
            const wide_t lhs = (point.x - ax) * (by - ay);
            const wide_t rhs = (bx - ax) * (point.y - ay);

            if ((by - ay) > 0 ? (lhs < rhs) : (lhs > rhs)) {
                inside = !inside;
            }
        }
    }

    return inside ? Location::Inside : Location::Outside;
}

// ---------------------------------------------------------------------------------------------
// Deterministic pseudo random numbers, so a failure is always reproducible
// ---------------------------------------------------------------------------------------------

class Random {
   public:
    explicit Random(uint64_t seed) : _state(seed) {}

    uint64_t next() {
        _state ^= _state << 13;
        _state ^= _state >> 7;
        _state ^= _state << 17;
        return _state;
    }

    // Both bounds inclusive
    int64_t inRange(int64_t from, int64_t to) {
        return from + (int64_t)(next() % (uint64_t)(to - from + 1));
    }

   private:
    uint64_t _state;
};

// ---------------------------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------------------------

template <typename T>
static std::vector<geo::Point<T>> toPoints(const std::vector<RefPoint>& polygon) {
    std::vector<geo::Point<T>> points;
    points.reserve(polygon.size());

    for (const RefPoint& point : polygon) {
        points.push_back(geo::Point<T>((T)point.x, (T)point.y));
    }

    return points;
}

/**
 * @brief Compare polygon.hpp against the reference for one point. Points on the outline are
 * accepted either way.
 */
template <typename T>
static void compareToReference(const geo::Polygon<T>& polygon, const std::vector<RefPoint>& outline,
                               const RefPoint& point, const std::string& what) {
    const Location expected = referenceLocation(outline, point);

    if (expected == Location::OnOutline) {
        return;
    }

    const bool got = polygon.checkPointInPolygon(geo::Point<T>((T)point.x, (T)point.y));
    const bool want = expected == Location::Inside;

    check(got == want, what + ": got " + (got ? "inside" : "outside") + ", expected " +
                           (want ? "inside" : "outside"));
}

// ---------------------------------------------------------------------------------------------
// Polygon generators
// ---------------------------------------------------------------------------------------------

/**
 * @brief A star shaped polygon around the origin. Alternating radii make it concave, so the ray
 * crosses many edges instead of just two.
 */
static std::vector<RefPoint> makeStar(int64_t radius, size_t corners) {
    std::vector<RefPoint> outline;

    for (size_t i = 0; i < corners * 2; ++i) {
        const double angle = 3.14159265358979323846 * (double)i / (double)corners;
        const double scale = (i % 2 == 0) ? 1.0 : 0.45;

        outline.push_back({(wide_t)(int64_t)((double)radius * scale * std::cos(angle)),
                           (wide_t)(int64_t)((double)radius * scale * std::sin(angle))});
    }

    return outline;
}

static std::vector<RefPoint> makeRectangle(int64_t xMin, int64_t yMin, int64_t xMax, int64_t yMax) {
    return {{(wide_t)xMin, (wide_t)yMin},
            {(wide_t)xMax, (wide_t)yMin},
            {(wide_t)xMax, (wide_t)yMax},
            {(wide_t)xMin, (wide_t)yMax}};
}

/**
 * @brief Random simple polygon: points sorted by angle around the origin, which can never
 * produce a self intersection.
 */
static std::vector<RefPoint> makeRandomSimple(Random& random, int64_t radius, size_t corners) {
    std::vector<RefPoint> outline;

    for (size_t i = 0; i < corners; ++i) {
        const double angle = 2.0 * 3.14159265358979323846 * (double)i / (double)corners;
        const double scale = 0.2 + 0.8 * (double)random.inRange(0, 1000) / 1000.0;

        outline.push_back({(wide_t)(int64_t)((double)radius * scale * std::cos(angle)),
                           (wide_t)(int64_t)((double)radius * scale * std::sin(angle))});
    }

    return outline;
}

// ---------------------------------------------------------------------------------------------
// Test cases, all of them run per coordinate type
// ---------------------------------------------------------------------------------------------

/**
 * @brief The regression this test file was written for: a polygon whose edges span a large part
 * of the coordinate range. Multiplying two such deltas does not fit into the coordinate type, so
 * an implementation that calculates in T silently reports points on the wrong side.
 */
template <typename T>
static void testLargeExtent(const char* typeName, int64_t extent) {
    const std::vector<RefPoint> outline = makeStar(extent, 5);
    const std::vector<geo::Point<T>> points = toPoints<T>(outline);
    const geo::Polygon<T> polygon(points.data(), points.size());

    Random random(0x9E3779B97F4A7C15ull);

    for (int i = 0; i < 4000; ++i) {
        const RefPoint point = {(wide_t)random.inRange(-extent, extent),
                                (wide_t)random.inRange(-extent, extent)};

        compareToReference(polygon, outline, point,
                           std::string(typeName) + " large extent star, sample " +
                               std::to_string(i));
    }
}

/**
 * @brief Same idea, but with the polygon pushed into a corner of the coordinate range so that
 * the differences themselves, not only their products, need the extra width.
 */
template <typename T>
static void testExtremeCoordinates(const char* typeName, int64_t limit) {
    const std::vector<RefPoint> outline =
        makeRectangle(-limit, -limit, limit - limit / 4, limit - limit / 4);
    const std::vector<geo::Point<T>> points = toPoints<T>(outline);
    const geo::Polygon<T> polygon(points.data(), points.size());

    const struct {
        int64_t x, y;
        bool inside;
    } samples[] = {
        {0, 0, true},
        {-limit / 2, limit / 2, true},
        {limit / 2, -limit / 2, true},
        {-limit + 1, -limit + 1, true},
        {limit - limit / 4 - 1, limit - limit / 4 - 1, true},
    };

    for (const auto& sample : samples) {
        const bool got = polygon.checkPointInPolygon(geo::Point<T>((T)sample.x, (T)sample.y));

        check(got == sample.inside, std::string(typeName) + " extreme rectangle (" +
                                        std::to_string(sample.x) + ", " +
                                        std::to_string(sample.y) + ")");
    }
}

/**
 * @brief Randomised cross check against the reference, over several random simple polygons.
 */
template <typename T>
static void testRandomPolygons(const char* typeName, int64_t radius) {
    Random random(0xD1B54A32D192ED03ull);

    for (size_t corners : {3u, 4u, 5u, 8u, 17u, 64u}) {
        const std::vector<RefPoint> outline = makeRandomSimple(random, radius, corners);
        const std::vector<geo::Point<T>> points = toPoints<T>(outline);
        const geo::Polygon<T> polygon(points.data(), points.size());

        for (int i = 0; i < 1500; ++i) {
            const RefPoint point = {(wide_t)random.inRange(-radius * 2, radius * 2),
                                    (wide_t)random.inRange(-radius * 2, radius * 2)};

            compareToReference(polygon, outline, point,
                               std::string(typeName) + " random polygon with " +
                                   std::to_string(corners) + " corners, sample " +
                                   std::to_string(i));
        }
    }
}

/**
 * @brief Probe right next to the outline, where the crossing arithmetic actually decides the
 * answer. For every edge the exact crossing of the ray is calculated in the reference type and
 * the points a few units to either side of it are checked. Uniform random sampling practically
 * never lands in that band, so this is where a rounding or overflow mistake hides.
 */
template <typename T>
static void testEdgeProximity(const char* typeName, int64_t radius) {
    Random random(0x2545F4914F6CDD1Dull);

    for (size_t corners : {3u, 5u, 9u, 24u}) {
        const std::vector<RefPoint> outline = makeRandomSimple(random, radius, corners);
        const std::vector<geo::Point<T>> points = toPoints<T>(outline);
        const geo::Polygon<T> polygon(points.data(), points.size());

        for (size_t i = 0, j = outline.size() - 1; i < outline.size(); j = i++) {
            const RefPoint& from = outline[j];
            const RefPoint& to = outline[i];

            if (from.y == to.y) {
                continue;
            }

            const wide_t low = from.y < to.y ? from.y : to.y;
            const wide_t high = from.y < to.y ? to.y : from.y;

            for (int sample = 0; sample < 40; ++sample) {
                const wide_t y = low + (wide_t)random.inRange(0, (int64_t)(high - low));
                const wide_t crossing =
                    from.x + (y - from.y) * (to.x - from.x) / (to.y - from.y);

                for (int offset = -3; offset <= 3; ++offset) {
                    compareToReference(polygon, outline, {crossing + offset, y},
                                       std::string(typeName) + " next to an edge of a " +
                                           std::to_string(corners) + " corner polygon");
                }
            }
        }
    }
}

/**
 * @brief A concave shape where the ray leaves and re-enters the polygon, plus points in the
 * notch that a convex-only implementation would get wrong.
 */
template <typename T>
static void testConcave(const char* typeName, int64_t unit) {
    // L shape, opening towards +x / +y
    const std::vector<RefPoint> outline = {{0, 0},
                                           {(wide_t)(4 * unit), 0},
                                           {(wide_t)(4 * unit), (wide_t)unit},
                                           {(wide_t)unit, (wide_t)unit},
                                           {(wide_t)unit, (wide_t)(4 * unit)},
                                           {0, (wide_t)(4 * unit)}};

    const std::vector<geo::Point<T>> points = toPoints<T>(outline);
    const geo::Polygon<T> polygon(points.data(), points.size());

    const struct {
        int64_t x, y;
        bool inside;
    } samples[] = {
        {unit / 2, unit / 2, true},          // corner of the L
        {3 * unit, unit / 2, true},          // foot of the L
        {unit / 2, 3 * unit, true},          // upright of the L
        {3 * unit, 3 * unit, false},         // the notch
        {2 * unit, 2 * unit, false},         // the notch
        {5 * unit, unit / 2, false},         // outside, beyond the foot
        {-unit, unit, false},                // outside, behind the start of the ray
    };

    for (const auto& sample : samples) {
        const bool got = polygon.checkPointInPolygon(geo::Point<T>((T)sample.x, (T)sample.y));

        check(got == sample.inside, std::string(typeName) + " L shape (" +
                                        std::to_string(sample.x) + ", " +
                                        std::to_string(sample.y) + ")");
    }
}

/**
 * @brief The smallest possible polygon. Three corners have to be accepted, and a triangle is
 * the only shape where the uniqueness count in isValidPolygon() has no slack.
 */
template <typename T>
static void testTriangle(const char* typeName, int64_t unit) {
    const geo::Point<T> corners[] = {geo::Point<T>(0, 0), geo::Point<T>((T)(4 * unit), 0),
                                     geo::Point<T>(0, (T)(4 * unit))};
    const geo::Polygon<T> triangle(corners, 3);

    check(triangle.isValidPolygon(), std::string(typeName) + " a triangle is a valid polygon");
    check(triangle.checkPointInPolygon(geo::Point<T>((T)unit, (T)unit)),
          std::string(typeName) + " triangle contains a point near its right angle");
    check(triangle.checkPointInPolygon(geo::Point<T>((T)(unit / 2), (T)(2 * unit))),
          std::string(typeName) + " triangle contains a point near its upright");
    check(!triangle.checkPointInPolygon(geo::Point<T>((T)(3 * unit), (T)(3 * unit))),
          std::string(typeName) + " triangle excludes a point beyond the hypotenuse");
    check(!triangle.checkPointInPolygon(geo::Point<T>((T)(-unit), (T)unit)),
          std::string(typeName) + " triangle excludes a point behind the start of the ray");
}

/**
 * @brief Degenerate inputs must not be reported as containing anything.
 */
template <typename T>
static void testDegenerate(const char* typeName, int64_t unit) {
    const geo::Point<T> twoPoints[] = {geo::Point<T>(0, 0), geo::Point<T>((T)unit, (T)unit)};
    const geo::Polygon<T> line(twoPoints, 2);

    check(!line.isValidPolygon(), std::string(typeName) + " two points are not a polygon");
    check(!line.checkPointInPolygon(geo::Point<T>(0, 0)),
          std::string(typeName) + " two points contain nothing");

    const geo::Point<T> repeated[] = {geo::Point<T>(0, 0), geo::Point<T>(0, 0),
                                      geo::Point<T>(0, 0), geo::Point<T>(0, 0)};
    const geo::Polygon<T> collapsed(repeated, 4);

    check(!collapsed.isValidPolygon(),
          std::string(typeName) + " repeated points are not a polygon");
    check(!collapsed.checkPointInPolygon(geo::Point<T>(0, 0)),
          std::string(typeName) + " repeated points contain nothing");
}

/**
 * @brief A repeated first point at the end has to be dropped, otherwise the outline gets an
 * extra zero length edge.
 */
template <typename T>
static void testClosedOutline(const char* typeName, int64_t unit) {
    const geo::Point<T> closed[] = {geo::Point<T>(0, 0), geo::Point<T>((T)unit, 0),
                                    geo::Point<T>((T)unit, (T)unit), geo::Point<T>(0, (T)unit),
                                    geo::Point<T>(0, 0)};
    const geo::Polygon<T> polygon(closed, 5);

    check(polygon.getCount() == 4, std::string(typeName) + " repeated last point is dropped");
    check(polygon.checkPointInPolygon(geo::Point<T>((T)(unit / 2), (T)(unit / 2))),
          std::string(typeName) + " closed outline contains its centre");
    check(!polygon.checkPointInPolygon(geo::Point<T>((T)(unit * 2), (T)(unit / 2))),
          std::string(typeName) + " closed outline excludes an outside point");
}

/**
 * @brief Everything outside the bounding box has to be rejected.
 */
template <typename T>
static void testBoundingBox(const char* typeName, int64_t unit) {
    const std::vector<RefPoint> outline = makeRectangle(-unit, -unit, unit, unit);
    const std::vector<geo::Point<T>> points = toPoints<T>(outline);
    const geo::Polygon<T> polygon(points.data(), points.size());

    check(polygon.boundingBox.xMin == (T)(-unit) && polygon.boundingBox.xMax == (T)unit &&
              polygon.boundingBox.yMin == (T)(-unit) && polygon.boundingBox.yMax == (T)unit,
          std::string(typeName) + " bounding box matches the outline");

    const int64_t outsides[][2] = {{unit * 2, 0}, {-unit * 2, 0}, {0, unit * 2},
                                   {0, -unit * 2}, {unit * 2, unit * 2}};

    for (const auto& outside : outsides) {
        check(!polygon.checkPointInPolygon(geo::Point<T>((T)outside[0], (T)outside[1])),
              std::string(typeName) + " outside the bounding box (" +
                  std::to_string(outside[0]) + ", " + std::to_string(outside[1]) + ")");
    }
}

// ---------------------------------------------------------------------------------------------

/**
 * @param extent largest coordinate magnitude the type can hold exactly for these tests
 */
template <typename T>
static void runAll(const char* typeName, int64_t extent) {
    std::printf("%s (extent %lld)\n", typeName, (long long)extent);

    const int before = g_failed;

    testLargeExtent<T>(typeName, extent);
    testExtremeCoordinates<T>(typeName, extent);
    testRandomPolygons<T>(typeName, extent / 2);
    testEdgeProximity<T>(typeName, extent / 2);
    testTriangle<T>(typeName, extent / 8);
    testConcave<T>(typeName, extent / 8);
    testDegenerate<T>(typeName, extent / 8);
    testClosedOutline<T>(typeName, extent / 8);
    testBoundingBox<T>(typeName, extent / 8);

    std::printf("  %s\n", g_failed == before ? "ok" : "FAILED");
}

int main() {
    // int16_t and int32_t hold every whole number up to their limit, float up to 2^24 and
    // double up to 2^53. int64_t is kept below 2^53 as well, so the same generators can be used.
    std::printf("intermediate type is %d bit\n\n", INTERMEDIATE_BITS);

    runAll<int16_t>("int16_t", exactExtent(30000));
    runAll<int32_t>("int32_t", exactExtent(2000000000));
    runAll<int64_t>("int64_t", exactExtent(4000000000000ll));
    runAll<float>("float", exactExtent(16000000));
    runAll<double>("double", exactExtent(4000000000000ll));

    // The coordinate convention this library is used with: degrees at 1e7 precision. A polygon
    // of continental size has edges whose delta product needs far more than 32 bits.
    {
        const std::vector<RefPoint> outline = {
            {-97756598, 1291341206},  {-201491614, 1099467126}, {-374887210, 1097887115},
            {-468842062, 1507205650}, {-330695678, 1605590494}, {-201796879, 1562535491},
            {-92454520, 1437942717},  {-96744003, 1422420813}};

        const std::vector<geo::Point<int32_t>> points = toPoints<int32_t>(outline);
        const geo::Polygon<int32_t> polygon(points.data(), points.size());

        std::printf("continent sized polygon at 1e7 degree precision\n");

        const int before = g_failed;
        int checked = 0;

        for (int64_t x = -500000000; x <= -50000000; x += 2500000) {
            for (int64_t y = 1050000000; y <= 1650000000; y += 2500000) {
                compareToReference(polygon, outline, {(wide_t)x, (wide_t)y},
                                   "1e7 grid (" + std::to_string(x) + ", " + std::to_string(y) +
                                       ")");
                ++checked;
            }
        }

        std::printf("  %d grid points, %s\n", checked, g_failed == before ? "ok" : "FAILED");
    }

    // The worst case the 1e7 degree convention can produce: a polygon spanning the whole globe.
    // Its bounding box is 180e7 by 360e7, so the products reach 6.5e18 and still fit in a 64 bit
    // intermediate. This has to hold on a 32 bit target too, where __int128 is unavailable.
    {
        const std::vector<RefPoint> outline = {{-900000000, -1800000000},
                                               {900000000, -1800000000},
                                               {900000000, 1800000000},
                                               {-900000000, 1800000000}};

        const std::vector<geo::Point<int32_t>> points = toPoints<int32_t>(outline);
        const geo::Polygon<int32_t> polygon(points.data(), points.size());

        std::printf("whole globe polygon at 1e7 degree precision\n");

        const int before = g_failed;
        Random random(0x14057B7EF767814Full);

        for (int i = 0; i < 20000; ++i) {
            compareToReference(polygon, outline,
                               {(wide_t)random.inRange(-900000000, 900000000),
                                (wide_t)random.inRange(-1800000000, 1800000000)},
                               "whole globe sample " + std::to_string(i));
        }

        // and hard against the corners, where the deltas are at their largest
        for (int64_t x : {-899999999ll, -450000000ll, 0ll, 450000000ll, 899999999ll}) {
            for (int64_t y : {-1799999999ll, -900000000ll, 0ll, 900000000ll, 1799999999ll}) {
                compareToReference(polygon, outline, {(wide_t)x, (wide_t)y}, "whole globe corner");
            }
        }

        std::printf("  %s\n", g_failed == before ? "ok" : "FAILED");
    }

    std::printf("\n%d checks, %d failed\n", g_checks, g_failed);
    return g_failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

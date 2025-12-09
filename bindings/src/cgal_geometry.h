#pragma once

// CGAL geometry operations for NEPath
// Replaces Clipper with CGAL for polygon offsetting and boolean operations
//
// Two offset methods available:
// 1. Straight skeleton (default) - fast, linear edges at corners
// 2. Minkowski sum with disc - true circular arcs, discretized to polyline

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <CGAL/Straight_skeleton_builder_2.h>

// Note: Minkowski-based round offset (approximated_offset_2) requires
// a kernel with exact rational arithmetic (e.g., CGAL::Lazy_exact_nt).
// For now, we only support straight skeleton offsetting which works
// with the simpler Exact_predicates_inexact_constructions_kernel.

#include <Eigen/Dense>
#include <vector>
#include <memory>

namespace nepath {
namespace cgal {

// CGAL kernel and types
using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_2 = K::Point_2;
using Polygon_2 = CGAL::Polygon_2<K>;
using Polygon_with_holes_2 = CGAL::Polygon_with_holes_2<K>;

// Offset method enumeration
// Note: MINKOWSKI_ROUND is not yet implemented (requires exact rational kernel)
enum class OffsetMethod {
    STRAIGHT_SKELETON,  // Fast, miter-like corners (default)
    MINKOWSKI_ROUND     // True round corners - NOT YET IMPLEMENTED
};

// Eigen matrix types (row-major for numpy compatibility)
using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// ============================================================================
// Conversion utilities
// ============================================================================

// Convert Eigen matrix (Nx2) to CGAL Polygon_2
inline Polygon_2 matrix_to_polygon(const RowMatrixXd& points) {
    Polygon_2 poly;
    for (Eigen::Index i = 0; i < points.rows(); ++i) {
        poly.push_back(Point_2(points(i, 0), points(i, 1)));
    }
    return poly;
}

// Convert CGAL Polygon_2 to Eigen matrix (Nx2)
inline RowMatrixXd polygon_to_matrix(const Polygon_2& poly) {
    RowMatrixXd points(poly.size(), 2);
    Eigen::Index i = 0;
    for (auto it = poly.vertices_begin(); it != poly.vertices_end(); ++it, ++i) {
        points(i, 0) = CGAL::to_double(it->x());
        points(i, 1) = CGAL::to_double(it->y());
    }
    return points;
}

// Convert vector of Eigen matrices to vector of CGAL polygons
inline std::vector<Polygon_2> matrices_to_polygons(const std::vector<RowMatrixXd>& matrices) {
    std::vector<Polygon_2> polygons;
    polygons.reserve(matrices.size());
    for (const auto& m : matrices) {
        polygons.push_back(matrix_to_polygon(m));
    }
    return polygons;
}

// Convert vector of CGAL polygons to vector of Eigen matrices
inline std::vector<RowMatrixXd> polygons_to_matrices(const std::vector<Polygon_2>& polygons) {
    std::vector<RowMatrixXd> matrices;
    matrices.reserve(polygons.size());
    for (const auto& p : polygons) {
        matrices.push_back(polygon_to_matrix(p));
    }
    return matrices;
}

// ============================================================================
// Polygon offsetting
// ============================================================================

// Offset a single polygon (inward if distance > 0, outward if distance < 0)
// Returns multiple polygons (offset may split into multiple components)
// method: STRAIGHT_SKELETON (fast, miter corners) or MINKOWSKI_ROUND (circular arcs)
// arc_tolerance: for MINKOWSKI_ROUND, controls arc discretization (smaller = more points)
std::vector<RowMatrixXd> offset_polygon(
    const RowMatrixXd& contour,
    double distance,
    OffsetMethod method = OffsetMethod::STRAIGHT_SKELETON,
    double arc_tolerance = 0.01
);

// Offset a polygon with holes
// contour: outer boundary (Nx2)
// holes: vector of hole boundaries (each Mx2)
// distance: offset distance (positive = inward, negative = outward)
std::vector<RowMatrixXd> offset_polygon_with_holes(
    const RowMatrixXd& contour,
    const std::vector<RowMatrixXd>& holes,
    double distance,
    OffsetMethod method = OffsetMethod::STRAIGHT_SKELETON,
    double arc_tolerance = 0.01
);


// ============================================================================
// Boolean operations
// ============================================================================

// Compute difference: subject - clips
// Returns vector of resulting polygons (may be multiple disconnected regions)
std::vector<RowMatrixXd> polygon_difference(
    const RowMatrixXd& subject,
    const std::vector<RowMatrixXd>& clips
);

// Compute union of multiple polygons
std::vector<RowMatrixXd> polygon_union(
    const std::vector<RowMatrixXd>& polygons
);

// Compute intersection of two polygons
std::vector<RowMatrixXd> polygon_intersection(
    const RowMatrixXd& polygon1,
    const RowMatrixXd& polygon2
);

// Check if polygon1 is fully contained within polygon2
bool polygon_contains(
    const RowMatrixXd& inner,
    const RowMatrixXd& outer
);

// ============================================================================
// Combined operations (matching NEPath ContourParallel API)
// ============================================================================

// Tool compensation: offset contour inward, holes outward, then subtract
// Equivalent to ContourParallel::tool_compensate
std::vector<RowMatrixXd> tool_compensate(
    const RowMatrixXd& contour,
    const std::vector<RowMatrixXd>& holes,
    double distance
);

// Offset and cut holes: offset contour, subtract any intersecting holes
// Equivalent to ContourParallel::OffsetClipper(contour, holes, ...)
std::vector<RowMatrixXd> offset_and_cut_holes(
    const RowMatrixXd& contour,
    const std::vector<RowMatrixXd>& holes,
    double distance
);

} // namespace cgal
} // namespace nepath

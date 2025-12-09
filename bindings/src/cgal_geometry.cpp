#include "cgal_geometry.h"

#include <CGAL/create_offset_polygons_2.h>
#include <CGAL/Polygon_set_2.h>
#include <cmath>
#include <stdexcept>

namespace nepath {
namespace cgal {

// ============================================================================
// Polygon offsetting - Straight Skeleton method
// ============================================================================

static std::vector<RowMatrixXd> offset_polygon_skeleton(
    const RowMatrixXd& contour,
    double distance
) {
    Polygon_2 poly = matrix_to_polygon(contour);

    // Ensure correct orientation (CCW for outer boundary)
    if (poly.is_clockwise_oriented()) {
        poly.reverse_orientation();
    }

    std::vector<RowMatrixXd> result;

    if (distance > 0) {
        // Inward offset using straight skeleton
        auto offset_polys = CGAL::create_interior_skeleton_and_offset_polygons_2(distance, poly);
        for (const auto& offset_poly : offset_polys) {
            if (offset_poly->size() >= 3) {
                Polygon_2 p;
                for (auto it = offset_poly->vertices_begin(); it != offset_poly->vertices_end(); ++it) {
                    p.push_back(Point_2(CGAL::to_double(it->x()), CGAL::to_double(it->y())));
                }
                result.push_back(polygon_to_matrix(p));
            }
        }
    } else if (distance < 0) {
        // Outward offset using exterior skeleton
        auto offset_polys = CGAL::create_exterior_skeleton_and_offset_polygons_2(-distance, poly);
        // Skip first polygon (it's the frame/bounding rectangle)
        for (size_t i = 1; i < offset_polys.size(); ++i) {
            const auto& offset_poly = offset_polys[i];
            if (offset_poly->size() >= 3) {
                Polygon_2 p;
                for (auto it = offset_poly->vertices_begin(); it != offset_poly->vertices_end(); ++it) {
                    p.push_back(Point_2(CGAL::to_double(it->x()), CGAL::to_double(it->y())));
                }
                result.push_back(polygon_to_matrix(p));
            }
        }
    } else {
        // Zero offset, return original
        result.push_back(contour);
    }

    return result;
}

// ============================================================================
// Main offset function
// ============================================================================

std::vector<RowMatrixXd> offset_polygon(
    const RowMatrixXd& contour,
    double distance,
    OffsetMethod method,
    double arc_tolerance
) {
    // Currently only STRAIGHT_SKELETON is implemented
    // MINKOWSKI_ROUND requires exact rational kernel (future work)
    if (method == OffsetMethod::MINKOWSKI_ROUND) {
        throw std::runtime_error(
            "MINKOWSKI_ROUND offset method is not yet implemented. "
            "Use STRAIGHT_SKELETON instead."
        );
    }
    (void)arc_tolerance; // Unused for now
    return offset_polygon_skeleton(contour, distance);
}

std::vector<RowMatrixXd> offset_polygon_with_holes(
    const RowMatrixXd& contour,
    const std::vector<RowMatrixXd>& holes,
    double distance,
    OffsetMethod method,
    double arc_tolerance
) {
    Polygon_2 outer = matrix_to_polygon(contour);

    // Ensure correct orientation (CCW for outer boundary)
    if (outer.is_clockwise_oriented()) {
        outer.reverse_orientation();
    }

    // Convert holes to CGAL polygons
    std::vector<Polygon_2> hole_polygons;
    for (const auto& hole_matrix : holes) {
        Polygon_2 hole = matrix_to_polygon(hole_matrix);
        // Holes should be CW oriented
        if (hole.is_counterclockwise_oriented()) {
            hole.reverse_orientation();
        }
        hole_polygons.push_back(hole);
    }

    std::vector<RowMatrixXd> result;

    if (distance > 0) {
        // Inward offset using straight skeleton with holes
        // Create Polygon_with_holes_2 and use the dedicated function
        Polygon_with_holes_2 pwh(outer);
        for (const auto& hole : hole_polygons) {
            pwh.add_hole(hole);
        }

        auto offset_pwh_vec = CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(distance, pwh);
        for (const auto& offset_pwh : offset_pwh_vec) {
            // Get outer boundary of each result polygon with holes
            const auto& outer_boundary = offset_pwh->outer_boundary();
            if (outer_boundary.size() >= 3) {
                result.push_back(polygon_to_matrix(outer_boundary));
            }
            // Note: We don't handle inner holes in the result here
            // Those would require a more complex return type
        }
    } else if (distance < 0) {
        // For outward offset with holes, we need a different approach
        // Offset the outer boundary outward
        auto outer_offsets = offset_polygon(contour, distance, method, arc_tolerance);

        // Offset each hole inward (which makes them bigger as obstacles)
        std::vector<RowMatrixXd> expanded_holes;
        for (const auto& hole : holes) {
            // Inward offset of hole = outward expansion of obstacle
            auto hole_offsets = offset_polygon(hole, -distance, method, arc_tolerance);
            for (const auto& ho : hole_offsets) {
                expanded_holes.push_back(ho);
            }
        }

        // Subtract expanded holes from outer offset
        for (const auto& outer_off : outer_offsets) {
            auto diff_result = polygon_difference(outer_off, expanded_holes);
            for (const auto& r : diff_result) {
                result.push_back(r);
            }
        }
    } else {
        result.push_back(contour);
    }

    return result;
}

// ============================================================================
// Boolean operations
// ============================================================================

std::vector<RowMatrixXd> polygon_difference(
    const RowMatrixXd& subject,
    const std::vector<RowMatrixXd>& clips
) {
    if (clips.empty()) {
        return {subject};
    }

    Polygon_2 subj_poly = matrix_to_polygon(subject);
    if (subj_poly.is_clockwise_oriented()) {
        subj_poly.reverse_orientation();
    }

    // Use polygon set for boolean operations
    CGAL::Polygon_set_2<K> ps;
    ps.insert(subj_poly);

    for (const auto& clip_matrix : clips) {
        Polygon_2 clip_poly = matrix_to_polygon(clip_matrix);
        if (clip_poly.is_clockwise_oriented()) {
            clip_poly.reverse_orientation();
        }
        ps.difference(clip_poly);
    }

    std::vector<RowMatrixXd> result;
    std::vector<Polygon_with_holes_2> result_pwh;
    ps.polygons_with_holes(std::back_inserter(result_pwh));

    for (const auto& pwh : result_pwh) {
        // Add outer boundary
        result.push_back(polygon_to_matrix(pwh.outer_boundary()));
        // Note: This simplified version doesn't return holes separately
        // In a full implementation, we'd return a more complex structure
    }

    return result;
}

std::vector<RowMatrixXd> polygon_union(
    const std::vector<RowMatrixXd>& polygons
) {
    if (polygons.empty()) {
        return {};
    }
    if (polygons.size() == 1) {
        return {polygons[0]};
    }

    CGAL::Polygon_set_2<K> ps;

    for (const auto& poly_matrix : polygons) {
        Polygon_2 poly = matrix_to_polygon(poly_matrix);
        if (poly.is_clockwise_oriented()) {
            poly.reverse_orientation();
        }
        ps.join(poly);
    }

    std::vector<RowMatrixXd> result;
    std::vector<Polygon_with_holes_2> result_pwh;
    ps.polygons_with_holes(std::back_inserter(result_pwh));

    for (const auto& pwh : result_pwh) {
        result.push_back(polygon_to_matrix(pwh.outer_boundary()));
    }

    return result;
}

std::vector<RowMatrixXd> polygon_intersection(
    const RowMatrixXd& polygon1,
    const RowMatrixXd& polygon2
) {
    Polygon_2 poly1 = matrix_to_polygon(polygon1);
    Polygon_2 poly2 = matrix_to_polygon(polygon2);

    if (poly1.is_clockwise_oriented()) poly1.reverse_orientation();
    if (poly2.is_clockwise_oriented()) poly2.reverse_orientation();

    CGAL::Polygon_set_2<K> ps;
    ps.insert(poly1);
    ps.intersection(poly2);

    std::vector<RowMatrixXd> result;
    std::vector<Polygon_with_holes_2> result_pwh;
    ps.polygons_with_holes(std::back_inserter(result_pwh));

    for (const auto& pwh : result_pwh) {
        result.push_back(polygon_to_matrix(pwh.outer_boundary()));
    }

    return result;
}

bool polygon_contains(
    const RowMatrixXd& inner,
    const RowMatrixXd& outer
) {
    // Check if inner is fully contained in outer by computing difference
    // If inner - outer is empty, inner is contained in outer
    auto diff = polygon_difference(inner, {outer});
    return diff.empty();
}

// ============================================================================
// Combined operations
// ============================================================================

std::vector<RowMatrixXd> tool_compensate(
    const RowMatrixXd& contour,
    const std::vector<RowMatrixXd>& holes,
    double distance
) {
    // Offset contour inward (positive distance)
    auto contour_offset = offset_polygon(contour, distance);

    if (holes.empty()) {
        return contour_offset;
    }

    // Offset holes outward (negative distance from hole perspective = inward from material)
    std::vector<RowMatrixXd> holes_offset;
    for (const auto& hole : holes) {
        auto hole_off = offset_polygon(hole, -distance);
        for (const auto& h : hole_off) {
            holes_offset.push_back(h);
        }
    }

    // Subtract offset holes from offset contour
    std::vector<RowMatrixXd> result;
    for (const auto& c : contour_offset) {
        auto diff = polygon_difference(c, holes_offset);
        for (const auto& d : diff) {
            result.push_back(d);
        }
    }

    return result;
}

std::vector<RowMatrixXd> offset_and_cut_holes(
    const RowMatrixXd& contour,
    const std::vector<RowMatrixXd>& holes,
    double distance
) {
    // First offset the contour
    auto contour_offset = offset_polygon(contour, distance);

    if (holes.empty()) {
        return contour_offset;
    }

    // Cut (subtract) any holes that intersect
    std::vector<RowMatrixXd> result;
    for (const auto& c : contour_offset) {
        // Only subtract holes that actually intersect
        std::vector<RowMatrixXd> intersecting_holes;
        for (const auto& hole : holes) {
            auto isect = polygon_intersection(c, hole);
            if (!isect.empty()) {
                intersecting_holes.push_back(hole);
            }
        }

        if (intersecting_holes.empty()) {
            result.push_back(c);
        } else {
            auto diff = polygon_difference(c, intersecting_holes);
            for (const auto& d : diff) {
                result.push_back(d);
            }
        }
    }

    return result;
}

} // namespace cgal
} // namespace nepath

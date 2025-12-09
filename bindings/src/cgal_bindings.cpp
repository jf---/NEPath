// Nanobind bindings for CGAL geometry module

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "cgal_geometry.h"

namespace nb = nanobind;
using namespace nepath::cgal;

NB_MODULE(_cgal, m) {
    m.doc() = "CGAL-based geometry operations for NEPath";

    // Offset method enum
    nb::enum_<OffsetMethod>(m, "OffsetMethod")
        .value("STRAIGHT_SKELETON", OffsetMethod::STRAIGHT_SKELETON,
               "Fast offset using straight skeleton (miter-like corners)")
        .value("MINKOWSKI_ROUND", OffsetMethod::MINKOWSKI_ROUND,
               "Round offset using Minkowski sum with disc (true circular arcs, discretized)");

    // =========================================================================
    // Polygon offsetting
    // =========================================================================

    m.def("offset_polygon", &offset_polygon,
        nb::arg("contour"),
        nb::arg("distance"),
        nb::arg("method") = OffsetMethod::STRAIGHT_SKELETON,
        nb::arg("arc_tolerance") = 0.01,
        R"doc(
        Offset a polygon inward or outward.

        Parameters
        ----------
        contour : ndarray (N, 2)
            Polygon vertices as Nx2 array of (x, y) coordinates.
        distance : float
            Offset distance. Positive = inward, negative = outward.
        method : OffsetMethod, optional
            STRAIGHT_SKELETON (default): Fast, creates miter-like corners.
            MINKOWSKI_ROUND: True round corners using Minkowski sum with disc.
        arc_tolerance : float, optional
            For MINKOWSKI_ROUND method, controls arc discretization.
            Smaller values = more points on circular arcs. Default 0.01.

        Returns
        -------
        list of ndarray
            List of offset polygons (offset may split into multiple components).
        )doc"
    );

    m.def("offset_polygon_with_holes", &offset_polygon_with_holes,
        nb::arg("contour"),
        nb::arg("holes"),
        nb::arg("distance"),
        nb::arg("method") = OffsetMethod::STRAIGHT_SKELETON,
        nb::arg("arc_tolerance") = 0.01,
        R"doc(
        Offset a polygon with holes.

        Parameters
        ----------
        contour : ndarray (N, 2)
            Outer boundary vertices.
        holes : list of ndarray
            List of hole boundaries, each as Mx2 array.
        distance : float
            Offset distance. Positive = inward, negative = outward.
        method : OffsetMethod, optional
            Offset method (see offset_polygon).
        arc_tolerance : float, optional
            Arc discretization tolerance for MINKOWSKI_ROUND method.

        Returns
        -------
        list of ndarray
            List of offset polygons.
        )doc"
    );

    // =========================================================================
    // Boolean operations
    // =========================================================================

    m.def("polygon_difference", &polygon_difference,
        nb::arg("subject"),
        nb::arg("clips"),
        R"doc(
        Compute boolean difference: subject - clips.

        Parameters
        ----------
        subject : ndarray (N, 2)
            Subject polygon vertices.
        clips : list of ndarray
            List of clip polygons to subtract.

        Returns
        -------
        list of ndarray
            Resulting polygons after subtraction.
        )doc"
    );

    m.def("polygon_union", &polygon_union,
        nb::arg("polygons"),
        R"doc(
        Compute boolean union of multiple polygons.

        Parameters
        ----------
        polygons : list of ndarray
            List of polygons to union.

        Returns
        -------
        list of ndarray
            Resulting union polygons.
        )doc"
    );

    m.def("polygon_intersection", &polygon_intersection,
        nb::arg("polygon1"),
        nb::arg("polygon2"),
        R"doc(
        Compute boolean intersection of two polygons.

        Parameters
        ----------
        polygon1 : ndarray (N, 2)
            First polygon.
        polygon2 : ndarray (M, 2)
            Second polygon.

        Returns
        -------
        list of ndarray
            Resulting intersection polygons.
        )doc"
    );

    m.def("polygon_contains", &polygon_contains,
        nb::arg("inner"),
        nb::arg("outer"),
        R"doc(
        Check if inner polygon is fully contained within outer polygon.

        Parameters
        ----------
        inner : ndarray (N, 2)
            Potentially contained polygon.
        outer : ndarray (M, 2)
            Containing polygon.

        Returns
        -------
        bool
            True if inner is fully contained in outer.
        )doc"
    );

    // =========================================================================
    // Combined operations (NEPath-style API)
    // =========================================================================

    m.def("tool_compensate", &tool_compensate,
        nb::arg("contour"),
        nb::arg("holes"),
        nb::arg("distance"),
        R"doc(
        Tool compensation: offset contour inward, holes outward, then subtract.

        This is the standard operation for preparing a slice for toolpath planning.
        The contour is offset inward by the tool radius, while holes are offset
        outward (making them larger obstacles).

        Parameters
        ----------
        contour : ndarray (N, 2)
            Outer boundary of the slice.
        holes : list of ndarray
            Interior holes.
        distance : float
            Tool compensation distance (typically half the line width).

        Returns
        -------
        list of ndarray
            Compensated polygons ready for toolpath planning.
        )doc"
    );

    m.def("offset_and_cut_holes", &offset_and_cut_holes,
        nb::arg("contour"),
        nb::arg("holes"),
        nb::arg("distance"),
        R"doc(
        Offset contour and subtract any intersecting holes.

        Parameters
        ----------
        contour : ndarray (N, 2)
            Contour to offset.
        holes : list of ndarray
            Holes to cut if they intersect.
        distance : float
            Offset distance. Positive = inward, negative = outward.

        Returns
        -------
        list of ndarray
            Offset polygons with holes cut out.
        )doc"
    );
}

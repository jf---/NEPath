"""
CGAL-based geometry operations for NEPath.

This module provides polygon offsetting and boolean operations using CGAL,
as an alternative to the Clipper-based operations in the main NEPath library.

Two offset methods are available:
- STRAIGHT_SKELETON: Fast offset using straight skeleton (miter-like corners)
- MINKOWSKI_ROUND: True round corners using Minkowski sum with disc

Example
-------
>>> import numpy as np
>>> from nepath_bindings.cgal import offset_polygon, OffsetMethod
>>>
>>> # Create a square
>>> square = np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float64)
>>>
>>> # Inward offset with straight skeleton (fast)
>>> result = offset_polygon(square, distance=1.0)
>>>
>>> # Outward offset with round corners
>>> result = offset_polygon(square, distance=-1.0, method=OffsetMethod.MINKOWSKI_ROUND)
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

try:
    from nepath_bindings._cgal import (
        OffsetMethod,
        offset_polygon as _offset_polygon,
        offset_polygon_with_holes as _offset_polygon_with_holes,
        polygon_difference as _polygon_difference,
        polygon_union as _polygon_union,
        polygon_intersection as _polygon_intersection,
        polygon_contains as _polygon_contains,
        tool_compensate as _tool_compensate,
        offset_and_cut_holes as _offset_and_cut_holes,
    )
    CGAL_AVAILABLE = True
except ImportError as e:
    CGAL_AVAILABLE = False
    OffsetMethod = None
    _import_error = str(e)


def _check_cgal():
    """Raise ImportError if CGAL is not available."""
    if not CGAL_AVAILABLE:
        raise ImportError(
            "CGAL geometry module is not available. "
            "Install CGAL and rebuild nepath_bindings with CGAL support."
        )


def offset_polygon(
    contour: NDArray[np.float64],
    distance: float,
    method: OffsetMethod = None,
    arc_tolerance: float = 0.01,
) -> list[NDArray[np.float64]]:
    """
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
    """
    _check_cgal()
    if method is None:
        method = OffsetMethod.STRAIGHT_SKELETON
    contour = np.asarray(contour, dtype=np.float64)
    return _offset_polygon(contour, distance, method, arc_tolerance)


def offset_polygon_with_holes(
    contour: NDArray[np.float64],
    holes: list[NDArray[np.float64]],
    distance: float,
    method: OffsetMethod = None,
    arc_tolerance: float = 0.01,
) -> list[NDArray[np.float64]]:
    """
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
    """
    _check_cgal()
    if method is None:
        method = OffsetMethod.STRAIGHT_SKELETON
    contour = np.asarray(contour, dtype=np.float64)
    holes = [np.asarray(h, dtype=np.float64) for h in holes]
    return _offset_polygon_with_holes(contour, holes, distance, method, arc_tolerance)


def polygon_difference(
    subject: NDArray[np.float64],
    clips: list[NDArray[np.float64]],
) -> list[NDArray[np.float64]]:
    """
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
    """
    _check_cgal()
    subject = np.asarray(subject, dtype=np.float64)
    clips = [np.asarray(c, dtype=np.float64) for c in clips]
    return _polygon_difference(subject, clips)


def polygon_union(
    polygons: list[NDArray[np.float64]],
) -> list[NDArray[np.float64]]:
    """
    Compute boolean union of multiple polygons.

    Parameters
    ----------
    polygons : list of ndarray
        List of polygons to union.

    Returns
    -------
    list of ndarray
        Resulting union polygons.
    """
    _check_cgal()
    polygons = [np.asarray(p, dtype=np.float64) for p in polygons]
    return _polygon_union(polygons)


def polygon_intersection(
    polygon1: NDArray[np.float64],
    polygon2: NDArray[np.float64],
) -> list[NDArray[np.float64]]:
    """
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
    """
    _check_cgal()
    polygon1 = np.asarray(polygon1, dtype=np.float64)
    polygon2 = np.asarray(polygon2, dtype=np.float64)
    return _polygon_intersection(polygon1, polygon2)


def polygon_contains(
    inner: NDArray[np.float64],
    outer: NDArray[np.float64],
) -> bool:
    """
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
    """
    _check_cgal()
    inner = np.asarray(inner, dtype=np.float64)
    outer = np.asarray(outer, dtype=np.float64)
    return _polygon_contains(inner, outer)


def tool_compensate(
    contour: NDArray[np.float64],
    holes: list[NDArray[np.float64]],
    distance: float,
) -> list[NDArray[np.float64]]:
    """
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
    """
    _check_cgal()
    contour = np.asarray(contour, dtype=np.float64)
    holes = [np.asarray(h, dtype=np.float64) for h in holes]
    return _tool_compensate(contour, holes, distance)


def offset_and_cut_holes(
    contour: NDArray[np.float64],
    holes: list[NDArray[np.float64]],
    distance: float,
) -> list[NDArray[np.float64]]:
    """
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
    """
    _check_cgal()
    contour = np.asarray(contour, dtype=np.float64)
    holes = [np.asarray(h, dtype=np.float64) for h in holes]
    return _offset_and_cut_holes(contour, holes, distance)


__all__ = [
    "CGAL_AVAILABLE",
    "OffsetMethod",
    "offset_polygon",
    "offset_polygon_with_holes",
    "polygon_difference",
    "polygon_union",
    "polygon_intersection",
    "polygon_contains",
    "tool_compensate",
    "offset_and_cut_holes",
]

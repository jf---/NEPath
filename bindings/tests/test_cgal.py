"""Tests for CGAL geometry module."""

import numpy as np
import pytest

from nepath_bindings.cgal import (
    CGAL_AVAILABLE,
    OffsetMethod,
    offset_polygon,
    offset_polygon_with_holes,
    polygon_difference,
    polygon_union,
    polygon_intersection,
    polygon_contains,
    tool_compensate,
    offset_and_cut_holes,
)

pytestmark = pytest.mark.skipif(not CGAL_AVAILABLE, reason="CGAL not available")


@pytest.fixture
def square():
    """A 10x10 square centered at (5,5)."""
    return np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.float64)


@pytest.fixture
def small_square():
    """A 4x4 square centered at (5,5) - used as hole."""
    return np.array([[3, 3], [7, 3], [7, 7], [3, 7]], dtype=np.float64)


@pytest.fixture
def triangle():
    """An equilateral-ish triangle."""
    return np.array([[0, 0], [10, 0], [5, 8.66]], dtype=np.float64)


class TestOffsetPolygon:
    def test_inward_offset_square(self, square):
        """Inward offset of square should shrink it."""
        result = offset_polygon(square, distance=1.0)
        assert len(result) == 1
        p = result[0]
        # Should be 8x8 square
        assert np.isclose(p[:, 0].min(), 1.0, atol=0.01)
        assert np.isclose(p[:, 0].max(), 9.0, atol=0.01)
        assert np.isclose(p[:, 1].min(), 1.0, atol=0.01)
        assert np.isclose(p[:, 1].max(), 9.0, atol=0.01)

    def test_outward_offset_square(self, square):
        """Outward offset of square should expand it."""
        result = offset_polygon(square, distance=-1.0)
        assert len(result) == 1
        p = result[0]
        # Should be 12x12 square
        assert np.isclose(p[:, 0].min(), -1.0, atol=0.01)
        assert np.isclose(p[:, 0].max(), 11.0, atol=0.01)

    def test_large_inward_offset_vanishes(self, square):
        """Large enough inward offset should make polygon vanish."""
        result = offset_polygon(square, distance=6.0)
        # 10x10 square with offset 6 should vanish (offset > half width)
        assert len(result) == 0

    def test_zero_offset(self, square):
        """Zero offset should return original polygon."""
        result = offset_polygon(square, distance=0.0)
        assert len(result) == 1
        np.testing.assert_array_almost_equal(result[0], square)


class TestOffsetPolygonWithHoles:
    def test_offset_with_hole(self, square, small_square):
        """Offset polygon with hole should produce correct result."""
        result = offset_polygon_with_holes(square, [small_square], distance=0.5)
        assert len(result) >= 1
        # Outer boundary should be offset inward
        # Inner hole boundary should also be offset

    def test_offset_no_holes(self, square):
        """Offset with empty holes list should work like simple offset."""
        result = offset_polygon_with_holes(square, [], distance=1.0)
        assert len(result) == 1


class TestBooleanOperations:
    def test_difference(self, square, small_square):
        """Difference should subtract clip from subject."""
        result = polygon_difference(square, [small_square])
        assert len(result) >= 1
        # Area should be reduced

    def test_difference_no_overlap(self, square):
        """Difference with non-overlapping clip should return original."""
        far_square = np.array([[100, 100], [110, 100], [110, 110], [100, 110]], dtype=np.float64)
        result = polygon_difference(square, [far_square])
        assert len(result) == 1

    def test_union_overlapping(self, square):
        """Union of overlapping polygons should merge them."""
        square2 = np.array([[5, 5], [15, 5], [15, 15], [5, 15]], dtype=np.float64)
        result = polygon_union([square, square2])
        assert len(result) == 1
        # Should have 8 vertices for L-shape
        assert result[0].shape[0] == 8

    def test_union_non_overlapping(self, square):
        """Union of non-overlapping polygons should return both."""
        far_square = np.array([[100, 100], [110, 100], [110, 110], [100, 110]], dtype=np.float64)
        result = polygon_union([square, far_square])
        assert len(result) == 2

    def test_intersection(self, square):
        """Intersection should return overlapping region."""
        square2 = np.array([[5, 5], [15, 5], [15, 15], [5, 15]], dtype=np.float64)
        result = polygon_intersection(square, square2)
        assert len(result) == 1
        p = result[0]
        # Intersection should be 5x5 square from (5,5) to (10,10)
        assert np.isclose(p[:, 0].min(), 5.0, atol=0.01)
        assert np.isclose(p[:, 0].max(), 10.0, atol=0.01)
        assert np.isclose(p[:, 1].min(), 5.0, atol=0.01)
        assert np.isclose(p[:, 1].max(), 10.0, atol=0.01)

    def test_intersection_no_overlap(self, square):
        """Intersection of non-overlapping polygons should be empty."""
        far_square = np.array([[100, 100], [110, 100], [110, 110], [100, 110]], dtype=np.float64)
        result = polygon_intersection(square, far_square)
        assert len(result) == 0


class TestPolygonContains:
    def test_contains_true(self, square, small_square):
        """Small square is contained in large square."""
        assert polygon_contains(small_square, square) is True

    def test_contains_false(self, square, small_square):
        """Large square is not contained in small square."""
        assert polygon_contains(square, small_square) is False

    def test_contains_same(self, square):
        """Polygon contains itself."""
        assert polygon_contains(square, square) is True


class TestToolCompensate:
    def test_tool_compensate_basic(self, square, small_square):
        """Tool compensation should offset boundaries correctly."""
        result = tool_compensate(square, [small_square], distance=0.5)
        assert len(result) >= 1


class TestOffsetAndCutHoles:
    def test_offset_and_cut(self, square, small_square):
        """Offset and cut holes should work."""
        result = offset_and_cut_holes(square, [small_square], distance=1.0)
        assert len(result) >= 1

"""Traversability footprint-filter behavior."""
import numpy as np

from s1_navigation.elevation_map_publisher import maximum_filter_3x3


def test_maximum_filter_inflates_cost_by_one_cell():
    costs = np.zeros((5, 5), dtype=np.float32)
    costs[2, 2] = 1.0

    filtered = maximum_filter_3x3(costs)

    expected = np.zeros((5, 5), dtype=np.float32)
    expected[1:4, 1:4] = 1.0
    np.testing.assert_array_equal(filtered, expected)


def test_maximum_filter_uses_worst_neighboring_cost():
    costs = np.array([[0.1, 0.2, 0.3],
                      [0.4, 0.5, 0.6],
                      [0.7, 0.8, 0.9]], dtype=np.float32)

    filtered = maximum_filter_3x3(costs)

    np.testing.assert_allclose(
        [filtered[0, 0], filtered[1, 1], filtered[2, 2]],
        [0.5, 0.9, 0.9])

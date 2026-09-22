"""Tests for the shared WGS 84 and UTM terrain-frame conversion."""

import pytest

from s1_navigation.utm_conversion import (
    latlon_to_local,
    latlon_to_utm,
    local_to_latlon,
)


ORIGIN = (38.42287240335025, -110.78495572815902)


def test_metadata_origin_projects_to_expected_utm_coordinate():
    easting, northing = latlon_to_utm(*ORIGIN)
    assert easting == pytest.approx(518771.39193143265, abs=0.001)
    assert northing == pytest.approx(4252757.177159773, abs=0.001)


def test_origin_is_local_zero():
    assert latlon_to_local(*ORIGIN, *ORIGIN) == pytest.approx((0.0, 0.0))


@pytest.mark.parametrize('east,north', [
    (10.0, 10.0),
    (-999.0, 999.0),
    (900.0, -750.0),
])
def test_local_round_trip(east, north):
    latitude, longitude = local_to_latlon(east, north, *ORIGIN)
    actual_east, actual_north = latlon_to_local(
        latitude, longitude, *ORIGIN)
    assert actual_east == pytest.approx(east, abs=1e-6)
    assert actual_north == pytest.approx(north, abs=1e-6)

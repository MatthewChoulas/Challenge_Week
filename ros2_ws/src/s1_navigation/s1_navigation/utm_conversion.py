"""Convert between WGS 84 latitude/longitude and the terrain's local grid."""

from pyproj import CRS, Transformer


WGS84_CRS = CRS.from_epsg(4326)
UTM_ZONE_12N_CRS = CRS.from_epsg(26912)

_TO_UTM = Transformer.from_crs(
    WGS84_CRS, UTM_ZONE_12N_CRS, always_xy=True)
_FROM_UTM = Transformer.from_crs(
    UTM_ZONE_12N_CRS, WGS84_CRS, always_xy=True)


def latlon_to_utm(latitude, longitude):
    """Return NAD83 / UTM zone 12N easting and northing in metres."""
    return _TO_UTM.transform(longitude, latitude)


def utm_to_latlon(easting, northing):
    """Return WGS 84 latitude and longitude for a UTM grid position."""
    longitude, latitude = _FROM_UTM.transform(easting, northing)
    return latitude, longitude


def latlon_to_local(latitude, longitude, origin_latitude, origin_longitude):
    """Return local x/y as UTM easting/northing offsets from the origin."""
    origin_easting, origin_northing = latlon_to_utm(
        origin_latitude, origin_longitude)
    easting, northing = latlon_to_utm(latitude, longitude)
    return easting - origin_easting, northing - origin_northing


def local_to_latlon(east, north, origin_latitude, origin_longitude):
    """Convert local UTM offsets from the origin to WGS 84 coordinates."""
    origin_easting, origin_northing = latlon_to_utm(
        origin_latitude, origin_longitude)
    return utm_to_latlon(origin_easting + east, origin_northing + north)

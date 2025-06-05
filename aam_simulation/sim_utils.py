import math
import numpy as np

def latlon_to_cartesian(lat: float, lon: float, ref_lat_deg: float) -> np.ndarray:
    """
    Convert (latitude, longitude) in degrees to a local Cartesian coordinate (x, y) in feet,
    using an equirectangular approximation around a reference latitude ref_lat_deg.
    """
    feet_per_degree_lat = 69 * 5280 # 69 miles x 5280 feet
    feet_per_degree_lon = feet_per_degree_lat * math.cos(math.radians(ref_lat_deg))

    x = lon * feet_per_degree_lon
    y = lat * feet_per_degree_lat
    return np.array([x, y], dtype=float)

def unit_vector(vec: np.ndarray) -> np.ndarray:
    """Return the unit (normalized) vector of the input. If zero, return zero vector."""
    norm = np.linalg.norm(vec)
    if norm < 1e-8:
        return np.zeros_like(vec)
    return vec / norm

def signed_angle_between(u: np.ndarray, v: np.ndarray) -> float:
    """
    Compute the signed 2D angle (in radians) from vector u to vector v.
    Returns value in range (−π,π], positive if v is CCW from u.
    """
    u2 = unit_vector(u)
    v2 = unit_vector(v)
    return math.atan2(v2[1], v2[0]) - math.atan2(u2[1], u2[0])
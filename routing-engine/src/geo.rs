//! Geometry helpers: fixed-point coordinates, distances, bearings, polyline encoding.

use bytemuck::{Pod, Zeroable};
use serde::{Deserialize, Serialize};

/// Fixed-point scale: 1e-7 degrees (~1.1 cm). Fits lat/lon in an `i32`.
pub const COORD_SCALE: f64 = 1e7;
pub const EARTH_RADIUS_M: f64 = 6_371_008.8;

/// A coordinate stored as fixed-point 1e-7 degrees. 8 bytes, `Pod` so it can live in the mmap'd file.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Pod, Zeroable, Serialize, Deserialize)]
pub struct Coord {
    pub lat: i32,
    pub lon: i32,
}

impl Coord {
    #[inline]
    pub fn from_deg(lat: f64, lon: f64) -> Self {
        Self {
            lat: (lat * COORD_SCALE).round() as i32,
            lon: (lon * COORD_SCALE).round() as i32,
        }
    }
    #[inline]
    pub fn lat(self) -> f64 {
        self.lat as f64 / COORD_SCALE
    }
    #[inline]
    pub fn lon(self) -> f64 {
        self.lon as f64 / COORD_SCALE
    }
}

/// Great-circle distance in meters.
#[inline]
pub fn haversine(a: Coord, b: Coord) -> f64 {
    let (lat1, lat2) = (a.lat().to_radians(), b.lat().to_radians());
    let dlat = lat2 - lat1;
    let dlon = (b.lon() - a.lon()).to_radians();
    let h = (dlat * 0.5).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon * 0.5).sin().powi(2);
    2.0 * EARTH_RADIUS_M * h.sqrt().asin()
}

/// Equirectangular distance in meters: within 0.1% of haversine below ~10 km and several
/// times cheaper (no inverse trig). Used for geometry walks where pieces are short.
#[inline]
pub fn fast_dist(a: Coord, b: Coord) -> f64 {
    const K: f64 = std::f64::consts::PI / 180.0 / COORD_SCALE * EARTH_RADIUS_M;
    let mid_lat = ((a.lat as f64 + b.lat as f64) * 0.5 / COORD_SCALE).to_radians();
    let dx = (b.lon - a.lon) as f64 * K * mid_lat.cos();
    let dy = (b.lat - a.lat) as f64 * K;
    (dx * dx + dy * dy).sqrt()
}

/// Initial bearing from `a` to `b`, degrees in [0, 360).
#[inline]
pub fn bearing(a: Coord, b: Coord) -> f64 {
    let (lat1, lat2) = (a.lat().to_radians(), b.lat().to_radians());
    let dlon = (b.lon() - a.lon()).to_radians();
    let y = dlon.sin() * lat2.cos();
    let x = lat1.cos() * lat2.sin() - lat1.sin() * lat2.cos() * dlon.cos();
    (y.atan2(x).to_degrees() + 360.0) % 360.0
}

/// Signed turn angle from bearing `in_b` to bearing `out_b`, in (-180, 180].
/// Positive = right turn, negative = left turn, 0 = straight.
#[inline]
pub fn turn_angle(in_b: f64, out_b: f64) -> f64 {
    let mut d = out_b - in_b;
    while d > 180.0 {
        d -= 360.0;
    }
    while d <= -180.0 {
        d += 360.0;
    }
    d
}

/// Local equirectangular projection to meters around a reference latitude.
/// Accurate enough for nearest-segment search inside a grid cell neighbourhood.
#[derive(Clone, Copy, Debug)]
pub struct LocalProjection {
    pub cos_lat: f64,
}

impl LocalProjection {
    pub fn new(ref_lat_deg: f64) -> Self {
        Self {
            cos_lat: ref_lat_deg.to_radians().cos(),
        }
    }
    /// Projected (x, y) in meters.
    #[inline]
    pub fn project(&self, c: Coord) -> (f64, f64) {
        let k = std::f64::consts::PI / 180.0 * EARTH_RADIUS_M;
        (c.lon() * k * self.cos_lat, c.lat() * k)
    }
}

/// Projects point `p` onto segment `a`-`b` (all in projected meters).
/// Returns (fraction along segment in [0,1], squared distance).
#[inline]
pub fn project_on_segment(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 <= 1e-12 {
        0.0
    } else {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
    };
    let (qx, qy) = (a.0 + t * dx, a.1 + t * dy);
    (t, (p.0 - qx).powi(2) + (p.1 - qy).powi(2))
}

/// Linear interpolation between two coordinates.
#[inline]
pub fn lerp(a: Coord, b: Coord, t: f64) -> Coord {
    Coord {
        lat: a.lat + ((b.lat - a.lat) as f64 * t).round() as i32,
        lon: a.lon + ((b.lon - a.lon) as f64 * t).round() as i32,
    }
}

/// Google encoded polyline with the given precision (5 or 6).
pub fn encode_polyline(coords: &[Coord], precision: u32) -> String {
    let factor = 10f64.powi(precision as i32);
    let mut out: Vec<u8> = Vec::with_capacity(coords.len() * 8);
    let (mut plat, mut plon) = (0i64, 0i64);
    for c in coords {
        let lat = (c.lat() * factor).round() as i64;
        let lon = (c.lon() * factor).round() as i64;
        encode_value(lat - plat, &mut out);
        encode_value(lon - plon, &mut out);
        plat = lat;
        plon = lon;
    }
    // Only ASCII bytes 63..=126 are produced.
    String::from_utf8(out).unwrap()
}

#[inline]
fn encode_value(v: i64, out: &mut Vec<u8>) {
    let mut v = if v < 0 { !(v << 1) } else { v << 1 };
    while v >= 0x20 {
        out.push(((v & 0x1f) | 0x20) as u8 + 63);
        v >>= 5;
    }
    out.push(v as u8 + 63);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polyline_matches_reference() {
        // Reference example from Google's polyline documentation.
        let pts = [
            Coord::from_deg(38.5, -120.2),
            Coord::from_deg(40.7, -120.95),
            Coord::from_deg(43.252, -126.453),
        ];
        assert_eq!(encode_polyline(&pts, 5), "_p~iF~ps|U_ulLnnqC_mqNvxq`@");
    }

    #[test]
    fn haversine_known_distance() {
        // Tel Aviv (Azrieli) -> Jerusalem (Old City) ~ 54 km straight line.
        let d = haversine(Coord::from_deg(32.0744, 34.7918), Coord::from_deg(31.7767, 35.2345));
        assert!((d - 54_000.0).abs() < 2_000.0, "{d}");
    }

    #[test]
    fn turn_angles() {
        assert_eq!(turn_angle(0.0, 90.0), 90.0);
        assert_eq!(turn_angle(0.0, 270.0), -90.0);
        assert_eq!(turn_angle(350.0, 10.0), 20.0);
        assert_eq!(turn_angle(10.0, 350.0), -20.0);
    }
}

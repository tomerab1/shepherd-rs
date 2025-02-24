use glam::Vec2;

pub fn haversine_distance(lat1: f32, lon1: f32, lat2: f32, lon2: f32) -> f32 {
    // Earth's radius in meters
    const EARTH_RADIUS: f32 = 6_371_000.0;

    // Convert degrees to radians.
    let phi1 = lat1.to_radians();
    let phi2 = lat2.to_radians();
    let delta_phi = (lat2 - lat1).to_radians();
    let delta_lambda = (lon2 - lon1).to_radians();

    // Haversine formula
    let a = (delta_phi / 2.0).sin().powi(2)
        + phi1.cos() * phi2.cos() * (delta_lambda / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());

    EARTH_RADIUS * c
}

pub fn calc_turn_cost(
    prev_lat: f32,
    prev_lon: f32,
    curr_lat: f32,
    curr_lon: f32,
    next_lat: f32,
    next_lon: f32,
) -> f32 {
    let v1 = Vec2::new(curr_lat - prev_lat, curr_lon - prev_lon).normalize();
    let v2 = Vec2::new(next_lat - curr_lat, next_lon - curr_lon).normalize();

    let dot = v1.dot(v2).clamp(-1.0, 1.0);

    let k = 1.0;
    let turn_multiplier = 1.0 + k * (1.0 - dot);

    turn_multiplier
}

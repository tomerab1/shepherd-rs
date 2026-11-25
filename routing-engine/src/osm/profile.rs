//! Car routing profile: which OSM ways are routable, in which direction, and how fast.
//!
//! The profile is the only place that interprets OSM tags. To add another vehicle type,
//! implement another function with the same signature and select it at import time.

/// Road class, used for default speeds and for turn instructions.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum RoadClass {
    Motorway = 0,
    Trunk = 1,
    Primary = 2,
    Secondary = 3,
    Tertiary = 4,
    Unclassified = 5,
    Residential = 6,
    LivingStreet = 7,
    Service = 8,
    Link = 9,
}

impl RoadClass {
    pub fn from_u8(v: u8) -> Self {
        use RoadClass::*;
        match v {
            0 => Motorway,
            1 => Trunk,
            2 => Primary,
            3 => Secondary,
            4 => Tertiary,
            5 => Unclassified,
            6 => Residential,
            7 => LivingStreet,
            8 => Service,
            _ => Link,
        }
    }
}

/// The result of evaluating one OSM way.
#[derive(Clone, Debug, PartialEq)]
pub struct WayProfile {
    pub forward: bool,
    pub backward: bool,
    /// Travel speed in km/h for each direction.
    pub speed_fwd: f32,
    pub speed_bwd: f32,
    pub class: RoadClass,
    pub roundabout: bool,
    pub name: Option<String>,
}

fn class_and_speed(highway: &str) -> Option<(RoadClass, f32)> {
    use RoadClass::*;
    Some(match highway {
        "motorway" => (Motorway, 90.0),
        "motorway_link" => (Link, 45.0),
        "trunk" => (Trunk, 85.0),
        "trunk_link" => (Link, 40.0),
        "primary" => (Primary, 65.0),
        "primary_link" => (Link, 30.0),
        "secondary" => (Secondary, 55.0),
        "secondary_link" => (Link, 25.0),
        "tertiary" => (Tertiary, 40.0),
        "tertiary_link" => (Link, 20.0),
        "unclassified" => (Unclassified, 25.0),
        "residential" => (Residential, 25.0),
        "living_street" => (LivingStreet, 10.0),
        "service" => (Service, 15.0),
        "road" => (Unclassified, 20.0),
        _ => return None,
    })
}

/// Parses an OSM `maxspeed` value into km/h. Returns `None` for unknown or `none`.
pub fn parse_maxspeed(v: &str) -> Option<f32> {
    let v = v.trim();
    if let Ok(n) = v.parse::<f32>() {
        return (n > 0.0).then_some(n);
    }
    let lower = v.to_ascii_lowercase();
    if let Some(n) = lower.strip_suffix("mph") {
        return n.trim().parse::<f32>().ok().map(|n| n * 1.609_344);
    }
    if let Some(n) = lower.strip_suffix("km/h").or_else(|| lower.strip_suffix("kmh")) {
        return n.trim().parse::<f32>().ok();
    }
    if lower == "walk" {
        return Some(5.0);
    }
    // Implicit values such as "IL:urban", "DE:rural", "GB:motorway".
    if let Some((_, kind)) = lower.split_once(':') {
        return match kind {
            "urban" => Some(50.0),
            "rural" => Some(80.0),
            "trunk" => Some(90.0),
            "motorway" => Some(110.0),
            "living_street" => Some(20.0),
            "zone30" | "zone:30" => Some(30.0),
            _ => None,
        };
    }
    None
}

/// Evaluates a way for car routing. `tags` yields `(key, value)` pairs.
pub fn car_profile<'a>(tags: impl Iterator<Item = (&'a str, &'a str)>) -> Option<WayProfile> {
    let mut highway = None;
    let mut oneway = None;
    let mut junction = None;
    let mut access = None;
    let mut motor = None;
    let mut maxspeed = None;
    let mut maxspeed_fwd = None;
    let mut maxspeed_bwd = None;
    let mut name = None;
    let mut name_en = None;
    let mut reference = None;
    let mut area = false;
    let mut impassable = false;

    for (k, v) in tags {
        match k {
            "highway" => highway = Some(v),
            "oneway" => oneway = Some(v),
            "junction" => junction = Some(v),
            "access" => access = Some(v),
            "motor_vehicle" | "motorcar" => {
                // The more specific key wins if both are present.
                if k == "motorcar" || motor.is_none() {
                    motor = Some(v)
                }
            }
            "maxspeed" => maxspeed = parse_maxspeed(v),
            "maxspeed:forward" => maxspeed_fwd = parse_maxspeed(v),
            "maxspeed:backward" => maxspeed_bwd = parse_maxspeed(v),
            "name" => name = Some(v),
            "name:en" => name_en = Some(v),
            "ref" => reference = Some(v),
            "area" => area = v == "yes",
            "impassable" => impassable = v == "yes",
            _ => {}
        }
    }

    let (class, default_speed) = class_and_speed(highway?)?;
    if area || impassable {
        return None;
    }

    let denied = |v: &str| matches!(v, "no" | "private" | "agricultural" | "forestry" | "emergency");
    match motor {
        Some(m) if denied(m) => return None,
        Some(_) => {}
        None => {
            if access.is_some_and(denied) {
                return None;
            }
        }
    }

    let roundabout = matches!(junction, Some("roundabout") | Some("circular"));
    let (forward, backward) = match oneway {
        Some("yes") | Some("true") | Some("1") => (true, false),
        Some("-1") | Some("reverse") => (false, true),
        Some("reversible") | Some("alternating") => return None,
        Some("no") | Some("false") | Some("0") => (true, true),
        _ if roundabout || class == RoadClass::Motorway => (true, false),
        _ => (true, true),
    };

    // Real travel speed is below the legal limit; 80% is a reasonable free-flow estimate.
    let effective = |limit: Option<f32>| limit.map(|l| (l * 0.8).max(5.0)).unwrap_or(default_speed);
    let speed_fwd = effective(maxspeed_fwd.or(maxspeed));
    let speed_bwd = effective(maxspeed_bwd.or(maxspeed));

    let name = name_en.or(name).or(reference).map(str::to_owned);

    Some(WayProfile {
        forward,
        backward,
        speed_fwd,
        speed_bwd,
        class,
        roundabout,
        name,
    })
}

/// Evaluates a way for walking. Pedestrians ignore one-way rules and turn restrictions, never
/// use motorways, and use trunk roads only where explicitly allowed.
pub fn foot_profile<'a>(tags: impl Iterator<Item = (&'a str, &'a str)>) -> Option<WayProfile> {
    let mut highway = None;
    let mut foot = None;
    let mut access = None;
    let mut sidewalk = None;
    let mut name = None;
    let mut name_en = None;
    let mut reference = None;
    let mut oneway_foot = false;
    let mut area = false;
    for (k, v) in tags {
        match k {
            "highway" => highway = Some(v),
            "foot" => foot = Some(v),
            "access" => access = Some(v),
            "sidewalk" => sidewalk = Some(v),
            "name" => name = Some(v),
            "name:en" => name_en = Some(v),
            "ref" => reference = Some(v),
            "oneway:foot" => oneway_foot = v == "yes",
            "area" => area = v == "yes",
            _ => {}
        }
    }
    let highway = highway?;
    let allowed = |v: Option<&str>| matches!(v, Some("yes" | "designated" | "permissive" | "official"));
    let denied = |v: Option<&str>| matches!(v, Some("no" | "private" | "use_sidepath"));
    if denied(foot) || (denied(access) && !allowed(foot)) {
        return None;
    }
    let speed = match highway {
        "footway" | "pedestrian" | "living_street" | "residential" | "service" | "unclassified"
        | "tertiary" | "tertiary_link" | "secondary" | "secondary_link" | "primary" | "primary_link"
        | "road" | "corridor" | "crossing" => 5.0,
        "path" | "track" | "cycleway" | "bridleway" => 4.5,
        "steps" => 2.5,
        "trunk" | "trunk_link" => {
            if allowed(foot) || matches!(sidewalk, Some("both" | "left" | "right" | "yes" | "separate")) {
                5.0
            } else {
                return None;
            }
        }
        _ => return None,
    };
    if area && highway != "pedestrian" {
        return None;
    }
    Some(WayProfile {
        forward: true,
        backward: !oneway_foot,
        speed_fwd: speed,
        speed_bwd: speed,
        // Pedestrian instructions never talk about ramps or roundabout exits.
        class: RoadClass::Residential,
        roundabout: false,
        name: name_en.or(name).or(reference).map(str::to_owned),
    })
}

/// A routing profile: how tags become ways, and how turns cost.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Profile {
    Car,
    Foot,
}

impl Profile {
    pub fn parse(s: &str) -> Option<Profile> {
        match s {
            "car" | "driving" | "drive" => Some(Profile::Car),
            "foot" | "walking" | "walk" => Some(Profile::Foot),
            _ => None,
        }
    }
    pub fn name(self) -> &'static str {
        match self {
            Profile::Car => "driving",
            Profile::Foot => "walking",
        }
    }
    pub fn way<'a>(self, tags: impl Iterator<Item = (&'a str, &'a str)>) -> Option<WayProfile> {
        match self {
            Profile::Car => car_profile(tags),
            Profile::Foot => foot_profile(tags),
        }
    }
    /// Route on a node-based graph (junctions as nodes). Only valid for profiles without turn
    /// restrictions or turn costs, where it gives identical routes with far less memory.
    pub fn node_based(self) -> bool {
        self == Profile::Foot
    }

    /// Whether OSM turn restrictions apply (they are for vehicles).
    pub fn uses_restrictions(self) -> bool {
        self == Profile::Car
    }
    /// Default expected wait at a traffic light, in seconds, when no timing data is known.
    /// For a fixed-time signal with cycle C and effective green g, the uniform delay is
    /// C·(1−g/C)²/2 (Webster); a typical urban cycle of 90 s with 45% green gives ~12 s.
    /// Mid-block lights are usually pedestrian crossings that cars rarely stop long at.
    pub fn signal_delay_s(self, at_intersection: bool) -> f64 {
        match (self, at_intersection) {
            (Profile::Car, true) => 12.0,
            (Profile::Car, false) => 5.0,
            (Profile::Foot, _) => 15.0,
        }
    }
    /// (U-turns allowed at every junction?, U-turn cost in seconds). Pedestrians may turn
    /// anywhere, but with no turn costs a U-turn at a junction can never be on a shortest
    /// path, and turning around mid-street is covered by snapping to both directions, so
    /// both profiles only keep U-turn arcs at dead ends — far fewer arcs and CCH edges.
    pub fn u_turn(self) -> (bool, f64) {
        match self {
            Profile::Car => (false, 20.0),
            Profile::Foot => (false, 1.0),
        }
    }
    pub fn turn_penalty_s(self, angle_deg: f64) -> f64 {
        match self {
            Profile::Car => car_turn_penalty_s(angle_deg),
            Profile::Foot => 0.0,
        }
    }
}

/// Turn penalty in seconds for a signed turn angle (positive = right).
/// Right-hand traffic: left turns cross oncoming traffic and cost more.
pub fn car_turn_penalty_s(angle_deg: f64) -> f64 {
    let a = angle_deg.abs();
    if a < 30.0 {
        return 0.0;
    }
    // Smooth curve: up to ~7.5 s for a sharp turn.
    let base = 7.5 / (1.0 + (-((a - 90.0) / 20.0)).exp());
    if angle_deg < 0.0 {
        base * 1.3
    } else {
        base
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn eval(tags: &[(&'static str, &'static str)]) -> Option<WayProfile> {
        car_profile(tags.iter().copied())
    }

    #[test]
    fn maxspeed_variants() {
        assert_eq!(parse_maxspeed("50"), Some(50.0));
        assert_eq!(parse_maxspeed("30 mph").map(|v| v.round()), Some(48.0));
        assert_eq!(parse_maxspeed("IL:urban"), Some(50.0));
        assert_eq!(parse_maxspeed("none"), None);
    }

    #[test]
    fn oneway_handling() {
        let w = eval(&[("highway", "residential"), ("oneway", "-1")]).unwrap();
        assert!(!w.forward && w.backward);
        let w = eval(&[("highway", "motorway")]).unwrap();
        assert!(w.forward && !w.backward);
        let w = eval(&[("highway", "primary"), ("junction", "roundabout")]).unwrap();
        assert!(w.forward && !w.backward && w.roundabout);
        let w = eval(&[("highway", "motorway"), ("oneway", "no")]).unwrap();
        assert!(w.forward && w.backward);
    }

    #[test]
    fn access_rules() {
        assert!(eval(&[("highway", "service"), ("access", "private")]).is_none());
        assert!(eval(&[("highway", "service"), ("access", "private"), ("motorcar", "yes")]).is_some());
        assert!(eval(&[("highway", "footway")]).is_none());
        assert!(eval(&[("highway", "primary"), ("motor_vehicle", "no")]).is_none());
    }

    #[test]
    fn left_costs_more_than_right() {
        assert!(car_turn_penalty_s(-90.0) > car_turn_penalty_s(90.0));
        assert_eq!(car_turn_penalty_s(5.0), 0.0);
    }

    #[test]
    fn foot_rules() {
        let f = |t: &[(&'static str, &'static str)]| foot_profile(t.iter().copied());
        assert!(f(&[("highway", "motorway")]).is_none());
        assert!(f(&[("highway", "trunk")]).is_none());
        assert!(f(&[("highway", "trunk"), ("sidewalk", "both")]).is_some());
        let w = f(&[("highway", "residential"), ("oneway", "yes")]).unwrap();
        assert!(w.forward && w.backward, "pedestrians ignore oneway");
        assert!(f(&[("highway", "footway"), ("foot", "no")]).is_none());
        assert_eq!(f(&[("highway", "steps")]).unwrap().speed_fwd, 2.5);
    }
}

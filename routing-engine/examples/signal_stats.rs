//! Statistics about traffic-signal tagging in a PBF (diagnostics).
use osmpbf::{Element, ElementReader};
use std::collections::HashMap;

fn main() -> anyhow::Result<()> {
    let path = std::env::args().nth(1).expect("pbf path");
    let mut tags: HashMap<String, usize> = HashMap::new();
    let mut coords: Vec<(f64, f64)> = Vec::new();
    let mut total = 0usize;
    ElementReader::from_path(&path)?.for_each(|el| {
        let (t, lat, lon): (Vec<(String, String)>, f64, f64) = match &el {
            Element::DenseNode(n) => (n.tags().map(|(k, v)| (k.into(), v.into())).collect(), n.lat(), n.lon()),
            Element::Node(n) => (n.tags().map(|(k, v)| (k.into(), v.into())).collect(), n.lat(), n.lon()),
            _ => return,
        };
        let sig = t.iter().any(|(k, v)| k == "highway" && v == "traffic_signals");
        let crossing_sig = t.iter().any(|(k, v)| (k == "crossing" && v == "traffic_signals") || (k == "highway" && v == "crossing"));
        if sig || crossing_sig {
            if sig {
                total += 1;
                coords.push((lat, lon));
            }
            for (k, v) in &t {
                if ["traffic_signals", "traffic_signals:direction", "direction", "crossing", "highway", "traffic_signals:sound"].contains(&k.as_str()) {
                    *tags.entry(format!("{}{k}={v}", if sig { "[sig] " } else { "[xing] " })).or_default() += 1;
                }
            }
        }
    })?;
    let mut v: Vec<_> = tags.into_iter().collect();
    v.sort_by(|a, b| b.1.cmp(&a.1));
    println!("highway=traffic_signals nodes: {total}");
    for (k, c) in v.iter().take(25) {
        println!("{c:>7} {k}");
    }
    // Clusters: signals within 40 m of another signal.
    coords.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let mut near = 0;
    for i in 0..coords.len() {
        let mut j = i + 1;
        while j < coords.len() && (coords[j].0 - coords[i].0) * 111_000.0 < 40.0 {
            let dx = (coords[j].1 - coords[i].1) * 111_000.0 * coords[i].0.to_radians().cos();
            let dy = (coords[j].0 - coords[i].0) * 111_000.0;
            if (dx * dx + dy * dy).sqrt() < 40.0 {
                near += 1;
                break;
            }
            j += 1;
        }
    }
    println!("signals with another signal within 40 m: {near} of {total}");
    Ok(())
}

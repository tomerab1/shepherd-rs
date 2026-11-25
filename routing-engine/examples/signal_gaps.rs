//! How many signalised intersections are only visible through crossing lights.
use osmpbf::{Element, ElementReader};

fn main() -> anyhow::Result<()> {
    let path = std::env::args().nth(1).expect("pbf path");
    let mut veh: Vec<(f64, f64)> = Vec::new();
    let mut xing: Vec<(f64, f64)> = Vec::new();
    ElementReader::from_path(&path)?.for_each(|el| {
        let (t, lat, lon): (Vec<(String, String)>, f64, f64) = match &el {
            Element::DenseNode(n) => (n.tags().map(|(k, v)| (k.into(), v.into())).collect(), n.lat(), n.lon()),
            Element::Node(n) => (n.tags().map(|(k, v)| (k.into(), v.into())).collect(), n.lat(), n.lon()),
            _ => return,
        };
        let v = t.iter().any(|(k, v)| k == "highway" && v == "traffic_signals");
        let x = t.iter().any(|(k, v)| k == "crossing" && v.split(';').any(|p| p == "traffic_signals"));
        if v {
            veh.push((lat, lon));
        } else if x {
            xing.push((lat, lon));
        }
    })?;
    veh.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let near = |p: (f64, f64), r: f64| {
        let lo = veh.partition_point(|q| q.0 < p.0 - r / 111_000.0);
        veh[lo..].iter().take_while(|q| q.0 <= p.0 + r / 111_000.0).any(|q| {
            let dx = (q.1 - p.1) * 111_000.0 * p.0.to_radians().cos();
            let dy = (q.0 - p.0) * 111_000.0;
            (dx * dx + dy * dy).sqrt() <= r
        })
    };
    let lonely = xing.iter().filter(|&&p| !near(p, 60.0)).count();
    println!("vehicle lights: {}  crossing lights: {}", veh.len(), xing.len());
    println!("crossing lights with NO vehicle light within 60 m: {lonely} ({:.0}%)", 100.0 * lonely as f64 / xing.len() as f64);
    Ok(())
}

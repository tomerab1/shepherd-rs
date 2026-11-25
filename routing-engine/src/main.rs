use clap::{Parser, Subcommand};
use routing_engine::geo::Coord;
use routing_engine::route::RouteOptions;
use routing_engine::server;
use std::path::PathBuf;
use std::time::Instant;

#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

#[derive(Parser)]
#[command(name = "shepherd", version, about = "Road routing engine (CCH on an edge-based graph)")]
struct Cli {
    /// Worker threads for preprocessing and customization (default: all cores).
    #[arg(long, global = true)]
    threads: Option<usize>,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Build a graph file from an OSM .pbf extract.
    Preprocess {
        input: PathBuf,
        output: PathBuf,
        /// `driving` (default) or `walking`.
        #[arg(long, default_value = "driving")]
        profile: String,
    },
    /// Run the HTTP server. Pass one graph per profile (e.g. a driving and a walking graph).
    Serve {
        #[arg(required = true)]
        graphs: Vec<PathBuf>,
        #[arg(long, default_value = "127.0.0.1:5000")]
        bind: String,
        /// Maximum concurrent route computations (default: number of cores).
        #[arg(long)]
        max_concurrent: Option<usize>,
        /// Avoidable area as `name=path.geojson`; routes with `exclude=name` stay out of it.
        #[arg(long = "avoid-area")]
        avoid_areas: Vec<String>,
    },
    /// Find a boundary relation in a PBF and write it as an avoidable-area GeoJSON.
    ExtractArea {
        pbf: PathBuf,
        /// List relations whose name contains this text.
        #[arg(long)]
        find: Option<String>,
        /// Relation id to extract.
        #[arg(long)]
        relation: Option<i64>,
        #[arg(long)]
        out: Option<PathBuf>,
        #[arg(long, default_value = "area")]
        name: String,
    },
    /// Compute one route and print it as JSON. Coordinates are `lon,lat`.
    Route {
        graph: PathBuf,
        #[arg(required = true, num_args = 2..)]
        coords: Vec<String>,
    },
    /// Benchmark random routes.
    Bench {
        graph: PathBuf,
        #[arg(long, default_value_t = 2000)]
        queries: usize,
    },
    /// Compare CCH distances and paths against plain Dijkstra.
    Verify {
        graph: PathBuf,
        #[arg(long, default_value_t = 20)]
        sources: usize,
        #[arg(long, default_value_t = 200)]
        targets: usize,
    },
}

fn parse(c: &str) -> anyhow::Result<Coord> {
    let (lon, lat) = c.split_once(',').ok_or_else(|| anyhow::anyhow!("expected lon,lat: {c}"))?;
    Ok(Coord::from_deg(lat.trim().parse()?, lon.trim().parse()?))
}

fn main() -> anyhow::Result<()> {
    tracing_subscriber::fmt().with_target(false).with_writer(std::io::stderr).init();
    let cli = Cli::parse();
    let threads = cli.threads.unwrap_or_else(|| std::thread::available_parallelism().map_or(4, |n| n.get()));
    rayon::ThreadPoolBuilder::new().num_threads(threads).build_global()?;

    match cli.cmd {
        Cmd::Preprocess { input, output, profile } => {
            let t = Instant::now();
            let profile = routing_engine::osm::profile::Profile::parse(&profile)
                .ok_or_else(|| anyhow::anyhow!("unknown profile '{profile}' (driving or walking)"))?;
            let name = input.file_name().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default();
            let g = routing_engine::build::preprocess_file(&input.to_string_lossy(), &name, profile)?;
            g.save(&output)?;
            tracing::info!("wrote {} in {:.1?}", output.display(), t.elapsed());
        }
        Cmd::Serve {
            graphs,
            bind,
            max_concurrent,
            avoid_areas,
        } => {
            let mut areas = Vec::new();
            for spec in avoid_areas {
                let (name, path) = spec
                    .split_once('=')
                    .ok_or_else(|| anyhow::anyhow!("--avoid-area expects name=path.geojson"))?;
                areas.push((name.to_string(), routing_engine::area::Area::from_geojson(path.as_ref())?));
            }
            let rt = tokio::runtime::Builder::new_multi_thread()
                .worker_threads(threads.min(4))
                .max_blocking_threads(threads * 2)
                .enable_all()
                .build()?;
            rt.block_on(server::serve(graphs, areas, bind, max_concurrent.unwrap_or(threads)))?;
        }
        Cmd::Route { graph, coords } => {
            let engine = server::load_engine(&graph, &[])?;
            let coords: Vec<Coord> = coords.iter().map(|c| parse(c)).collect::<Result<_, _>>()?;
            let t = Instant::now();
            let r = engine.route(&coords, &RouteOptions::default())?;
            eprintln!("route computed in {:.2?}", t.elapsed());
            println!("{}", serde_json::to_string_pretty(&r)?);
        }
        Cmd::Bench { graph, queries } => bench(&graph, queries)?,
        Cmd::ExtractArea {
            pbf,
            find,
            relation,
            out,
            name,
        } => {
            let pbf = pbf.to_string_lossy().into_owned();
            if let Some(q) = find {
                for r in routing_engine::area::find_relations(&pbf, &q)? {
                    let pick = |k: &str| r.tags.iter().find(|t| t.0 == k).map(|t| t.1.as_str()).unwrap_or("");
                    println!(
                        "{}\t{}\t{} | {} | admin_level={} boundary={}",
                        r.id,
                        pick("type"),
                        pick("name"),
                        pick("name:en"),
                        pick("admin_level"),
                        pick("boundary")
                    );
                }
            }
            if let Some(id) = relation {
                let area = routing_engine::area::relation_area(&pbf, id)?;
                let out = out.ok_or_else(|| anyhow::anyhow!("--out is required with --relation"))?;
                std::fs::write(&out, serde_json::to_vec(&area.to_geojson(&name))?)?;
                println!(
                    "wrote {} ({} rings, {} points)",
                    out.display(),
                    area.rings.len(),
                    area.rings.iter().map(Vec::len).sum::<usize>()
                );
            }
        }
        Cmd::Verify {
            graph,
            sources,
            targets,
        } => {
            let engine = server::load_engine(&graph, &[])?;
            let t = Instant::now();
            let bad = routing_engine::verify::verify(&engine, sources, targets, 0x5eed);
            println!("checked {} pairs in {:.1?}: {} mismatches", sources * targets, t.elapsed(), bad);
            if bad > 0 {
                std::process::exit(1);
            }
        }
    }
    Ok(())
}

fn bench(graph: &PathBuf, queries: usize) -> anyhow::Result<()> {
    use rayon::prelude::*;
    let engine = server::load_engine(graph, &[])?;
    let g = &engine.graph;
    // Random pairs of road positions (geometry points), so every query snaps successfully.
    let mut rng = 0x9e3779b97f4a7c15u64;
    let mut next = |n: usize| {
        rng ^= rng << 13;
        rng ^= rng >> 7;
        rng ^= rng << 17;
        (rng % n as u64) as usize
    };
    let mut pairs = Vec::with_capacity(queries);
    while pairs.len() < queries {
        let a = g.geometry[next(g.geometry.len())];
        let b = g.geometry[next(g.geometry.len())];
        pairs.push([a, b]);
    }
    let m = engine.metric();
    let snaps: Vec<_> = pairs
        .iter()
        .filter_map(|p| Some((engine.snap(p[0])?, engine.snap(p[1])?)))
        .collect();

    let t = Instant::now();
    let mut found = 0;
    for (a, b) in &snaps {
        if engine.leg(&m, a, b).is_some() {
            found += 1;
        }
    }
    let el = t.elapsed();
    println!(
        "query + unpack: {} routes, {} found, {:.1} µs/route (1 thread)",
        snaps.len(),
        found,
        el.as_secs_f64() * 1e6 / snaps.len() as f64
    );

    let opts = RouteOptions::default();
    let t = Instant::now();
    for p in &pairs {
        let _ = engine.route(p, &opts);
    }
    let el = t.elapsed();
    println!(
        "full route (snap + query + steps + geometry): {:.1} µs/route (1 thread)",
        el.as_secs_f64() * 1e6 / pairs.len() as f64
    );

    let t = Instant::now();
    pairs.par_iter().for_each(|p| {
        let _ = engine.route(p, &opts);
    });
    let el = t.elapsed();
    println!(
        "full route, all cores: {:.0} routes/s",
        pairs.len() as f64 / el.as_secs_f64()
    );

    let t = Instant::now();
    let (v, ms) = engine.update(|o| {
        for s in (0..g.segments.len() as u32).step_by(50) {
            o.factor.insert(s, [Some(1.5), Some(1.5)]);
        }
    });
    println!(
        "traffic update on {} segments: re-customized in {:.0} ms (total {:.0?}), metric v{v}",
        g.segments.len() / 50,
        ms,
        t.elapsed()
    );
    Ok(())
}

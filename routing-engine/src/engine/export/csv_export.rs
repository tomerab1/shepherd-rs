use super::export_provider::ExportProvider;
use crate::engine::preprocess::graph::Graph;
use csv::Writer;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};

pub struct CSVExport<'a> {
    pub graph: &'a Graph,
    pub path: String,
}

impl<'a> CSVExport<'a> {
    pub fn new(graph: &'a Graph, path: String) -> Self {
        Self { graph, path }
    }
}

impl<'a> ExportProvider for CSVExport<'a> {
    type ExportType = anyhow::Result<()>;

    fn export(&self) -> Self::ExportType {
        let nodes = self.graph.get_nodes();
        let mut writer = Writer::from_path(&self.path)?;

        let records: Vec<Vec<String>> = nodes
            .par_iter()
            .map(|node| {
                vec![
                    node.dense_id.to_string(),
                    node.osm_id.to_string(),
                    node.lat.to_string(),
                    node.lon.to_string(),
                ]
            })
            .collect();

        for record in records {
            writer.write_record(record)?;
        }

        Ok(())
    }
}

pub trait ExportProvider {
    type ExportType;

    fn export(&self) -> Self::ExportType;
}

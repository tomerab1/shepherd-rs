pub trait Visitable {
    type Output;

    fn visit(&self) -> Self::Output;
}

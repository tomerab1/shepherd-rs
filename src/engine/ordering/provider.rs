pub trait OrderingProvider {
    type Item;

    fn get_node_order() -> Self::Item;
}

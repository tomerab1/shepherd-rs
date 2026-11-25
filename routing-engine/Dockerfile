# Build:  docker build -t shepherd routing-engine
# Run:    docker run -p 5000:5000 -v $PWD/routing-engine/data:/data shepherd \
#             serve /data/il.graph /data/il-foot.graph --avoid-area judea-samaria=/data/judea-samaria.geojson
FROM rust:1.96-slim AS build
WORKDIR /src
COPY Cargo.toml Cargo.lock ./
COPY src ./src
RUN cargo build --release --locked && strip target/release/shepherd

FROM debian:bookworm-slim
RUN useradd --system --uid 10001 shepherd
COPY --from=build /src/target/release/shepherd /usr/local/bin/shepherd
USER shepherd
EXPOSE 5000
HEALTHCHECK --interval=30s --timeout=3s CMD ["/usr/local/bin/shepherd", "--version"]
ENTRYPOINT ["/usr/local/bin/shepherd"]
CMD ["serve", "/data/il.graph", "--bind", "0.0.0.0:5000"]

//! Re-exports of segment-Voronoi interior medial axis graph, maximum inscribed circle pole
//! extraction, and pruned 2D medial-axis chains from [`chip_geom2d`].

pub use chip_geom2d::medial::{
    MedialChain2D, MedialNode2D, extract_interior_medial_nodes, extract_medial_axis_chains,
    find_medial_inscribed_pole,
};

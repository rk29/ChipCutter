//! Re-exports of exact 2D arc-polyline offsetting, planar [`PocketRegion2D`] erosion/dilation,
//! Offset Region Trees, scanline clipping, and helical entry pole discovery from [`chip_geom2d`].

pub use chip_geom2d::offset::{
    HelicalEntryPole, OffsetRegionNode, PocketRegion2D, build_offset_region_tree,
    clip_scanline_to_region, find_helical_entry_pole, from_cavalier_polyline, offset_polyline,
    to_cavalier_polyline,
};

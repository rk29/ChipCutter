mod carvera;
mod generic;
mod milo;
mod voron_cascade;

pub use carvera::{
    builtin_carvera_air_machine, builtin_carvera_machine, builtin_carvera_z1_machine,
};
pub use generic::builtin_generic_machine;
pub use milo::{builtin_milo_1_5_machine, builtin_milo_2_0_machine};
pub use voron_cascade::builtin_voron_cascade_machine;

/// Built-in read-only machine ID for the Millennium Machines Milo v1.5 (LDO Kit).
pub const MILO_1_5_MACHINE_ID: &str = "milo-1.5";

/// Built-in read-only machine ID for the Millennium Machines Milo v2.0.
pub const MILO_2_0_MACHINE_ID: &str = "milo-2.0";

/// Built-in read-only machine ID for the Voron Cascade.
pub const VORON_CASCADE_MACHINE_ID: &str = "voron-cascade";

/// Built-in read-only machine ID for the Makera Carvera (original ATC).
pub const CARVERA_MACHINE_ID: &str = "carvera";

/// Built-in read-only machine ID for the Makera Carvera Air.
pub const CARVERA_AIR_MACHINE_ID: &str = "carvera-air";

/// Built-in read-only machine ID for the Makera Carvera Z1.
pub const CARVERA_Z1_MACHINE_ID: &str = "carvera-z1";

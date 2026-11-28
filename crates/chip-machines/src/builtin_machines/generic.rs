use crate::machine::{
    DEFAULT_MACHINE_ID, MachineDefinition, MachineKinematicsSpec, MachineRotaryDefaults,
    MachineSpindleSpec, MachineToolChangeKind, MachineTravelEnvelope,
};
use chip_tools::builtin::default_starter_tool_library;

/// Canonical read-only baseline machine (`generic`) preserving the existing starter tool library.
pub fn builtin_generic_machine() -> MachineDefinition {
    let starter = default_starter_tool_library();
    let mut def = MachineDefinition {
        schema_version: MachineDefinition::CURRENT_SCHEMA_VERSION,
        id: DEFAULT_MACHINE_ID.to_string(),
        name: "Generic 3/4-Axis CNC Mill".to_string(),
        manufacturer: Some("ChipCutter".to_string()),
        description: Some(
            "Baseline 3-axis and 4th-axis CNC mill with the standard metric/imperial starter tool library."
                .to_string(),
        ),
        is_builtin: true,
        travel_mm: MachineTravelEnvelope::default(),
        spindle: MachineSpindleSpec::default(),
        kinematics: MachineKinematicsSpec::default(),
        tool_change: MachineToolChangeKind::Manual,
        has_toolsetter: false,
        default_post_profile: "grbl".to_string(),
        rotary: Some(MachineRotaryDefaults {
            axis: "A".to_string(),
            max_stock_diameter_mm: 120.0,
            max_stock_length_mm: 250.0,
            chuck_diameter_mm: 65.0,
            chuck_length_mm: 20.0,
            default_post_profile_4axis: Some("linuxcnc_4axis".to_string()),
        }),
        tools: starter.tools,
    };
    def.sort_tools_deterministically();
    def
}

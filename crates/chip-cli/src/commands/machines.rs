use std::{path::PathBuf, process::ExitCode};

use chip_machines::{
    clone_machine_definition, export_machine_to_path, import_machine_from_path,
    load_available_machines, resolve_machine_definition_cli_override, save_user_machine,
};

pub fn run_machines_list() -> Result<ExitCode, Box<dyn std::error::Error>> {
    let machines = load_available_machines(None);
    println!(
        "{:<16} {:<10} {:<20} {:<15} {:<7} {:<14} {:<6} Name",
        "ID", "Source", "Travel (mm)", "Spindle (RPM)", "Tools", "Default Post", "4th"
    );
    for m in &machines {
        let src = if m.is_builtin { "built-in" } else { "custom" };
        let travel = format!(
            "{:.0}x{:.0}x{:.0}",
            m.travel_mm.x_mm, m.travel_mm.y_mm, m.travel_mm.z_mm
        );
        let rpm = format!("{:.0}-{:.0}", m.spindle.min_rpm, m.spindle.max_rpm);
        let rotary = m
            .rotary
            .as_ref()
            .map_or_else(|| "no".to_string(), |r| format!("{}-axis", r.axis));
        println!(
            "{:<16} {:<10} {:<20} {:<15} {:<7} {:<14} {:<6} {}",
            m.id,
            src,
            travel,
            rpm,
            m.tools.len(),
            m.default_post_profile,
            rotary,
            m.name
        );
    }
    Ok(ExitCode::SUCCESS)
}

pub fn run_machines_inspect(id_or_path: String) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let m = resolve_machine_definition_cli_override(&id_or_path, None)?;
    println!("=== Machine Definition: '{}' ({}) ===", m.name, m.id);
    println!(
        "Source:       {}",
        if m.is_builtin {
            "built-in (read-only; clone to customize)"
        } else {
            "custom (user config)"
        }
    );
    if let Some(mfr) = &m.manufacturer {
        println!("Manufacturer: {mfr}");
    }
    if let Some(desc) = &m.description {
        println!("Description:  {desc}");
    }
    println!(
        "Travel XYZ:   {:.1} × {:.1} × {:.1} mm",
        m.travel_mm.x_mm, m.travel_mm.y_mm, m.travel_mm.z_mm
    );
    println!(
        "Spindle:      {:.0}–{:.0} RPM | Collet: {}{}",
        m.spindle.min_rpm,
        m.spindle.max_rpm,
        m.spindle.collet_type,
        m.spindle
            .power_watts
            .map_or(String::new(), |w| format!(" | {w:.0} W"))
    );
    println!(
        "Kinematics:   Rapid XY/Z={:.0}/{:.0} mm/min | Max Cut XY/Z={:.0}/{:.0} mm/min | Accel={:.0} mm/s²",
        m.kinematics.max_rapid_xy_mm_min,
        m.kinematics.max_rapid_z_mm_min,
        m.kinematics.max_cut_feed_xy_mm_min,
        m.kinematics.max_cut_feed_z_mm_min,
        m.kinematics.accel_mm_s2
    );
    println!(
        "Tool Change:  {} | Toolsetter: {} | Default Post: {}",
        m.tool_change.label(),
        if m.has_toolsetter { "yes" } else { "no" },
        m.default_post_profile
    );
    if let Some(rot) = &m.rotary {
        println!(
            "4th-Axis:     {}-Axis | Max Stock Ø{:.1} × {:.1} mm | Chuck Ø{:.1} × {:.1} mm | 4-Axis Post: {}",
            rot.axis,
            rot.max_stock_diameter_mm,
            rot.max_stock_length_mm,
            rot.chuck_diameter_mm,
            rot.chuck_length_mm,
            rot.default_post_profile_4axis
                .as_deref()
                .unwrap_or(&m.default_post_profile)
        );
    } else {
        println!("4th-Axis:     Disabled (3-axis machine)");
    }

    println!("Tools ({}):", m.tools.len());
    for t in &m.tools {
        println!(
            "  T{:<2} {:<44} {} | Ø{:.3} mm | LOC {:.1} mm | Shank Ø{:.3} mm | {} preset(s)",
            t.tool_number,
            t.name,
            t.geometry.kind_label(),
            t.geometry.cutting_diameter_mm(),
            t.geometry.flute_length_mm(),
            t.geometry.shank_diameter_mm(),
            t.presets.len()
        );
    }
    Ok(ExitCode::SUCCESS)
}

pub fn run_machines_clone(
    source_id: String,
    new_id: String,
    name: Option<String>,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let source = resolve_machine_definition_cli_override(&source_id, None)?;
    let display_name = name.unwrap_or_else(|| format!("{} (Custom)", source.name));
    let cloned = clone_machine_definition(&source, &new_id, display_name)?;
    let saved = save_user_machine(&cloned)?;
    println!(
        "Cloned machine '{}' -> '{}' ({} tools) to {}",
        source.id,
        cloned.id,
        cloned.tools.len(),
        saved.display()
    );
    Ok(ExitCode::SUCCESS)
}

pub fn run_machines_export(
    id_or_path: String,
    output: PathBuf,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let machine = resolve_machine_definition_cli_override(&id_or_path, None)?;
    export_machine_to_path(&machine, &output)?;
    println!(
        "Exported machine '{}' ({} tools) to {}",
        machine.id,
        machine.tools.len(),
        output.display()
    );
    Ok(ExitCode::SUCCESS)
}

pub fn run_machines_import(
    input: PathBuf,
    id_override: Option<String>,
) -> Result<ExitCode, Box<dyn std::error::Error>> {
    let imported = import_machine_from_path(&input, id_override.as_deref())?;
    println!(
        "Imported machine '{}' ('{}', {} tools) into user configuration.",
        imported.id,
        imported.name,
        imported.tools.len()
    );
    Ok(ExitCode::SUCCESS)
}

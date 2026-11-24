#![no_std]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

extern crate alloc;

use alloc::collections::BTreeSet;
use alloc::format;
use alloc::string::{String, ToString};
use serde::Deserialize;
use serde_json::{Value, json};

#[global_allocator]
static ALLOCATOR: dlmalloc::GlobalDlmalloc = dlmalloc::GlobalDlmalloc;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    core::arch::wasm32::unreachable()
}

wit_bindgen::generate!({
    path: "../wit/plugin.wit",
    world: "command-plugin",
});

use craft::plugin::commands;
use craft::plugin::control;
use craft::plugin::types::{FieldFailure, PluginError};
use exports::craft::plugin::entry::Guest;

struct GearSuitePlugin;

export!(GearSuitePlugin);

impl Guest for GearSuitePlugin {
    fn execute(entry: String, params: String) -> Result<String, PluginError> {
        let value: Value =
            serde_json::from_str(&params).map_err(|e| invalid("params", format!("invalid JSON: {e}")))?;
        match entry.as_str() {
            "parametric-spur-gear" => run_spur_gear(&value),
            "parametric-parallel-pair" => run_parallel_pair(&value),
            _ => Err(invalid("entry", format!("unknown entry `{entry}`"))),
        }
    }
}

fn invalid(field: &str, reason: impl Into<String>) -> PluginError {
    PluginError::InvalidInput(FieldFailure {
        field: field.to_string(),
        reason: reason.into(),
    })
}

fn run_cmd(cmd: &str, args: Value) -> Result<Value, PluginError> {
    if control::cancelled() {
        return Err(PluginError::Cancelled);
    }
    let raw = commands::run(cmd, &args.to_string())?;
    serde_json::from_str(&raw).map_err(|e| PluginError::Trap(format!("invalid JSON from {cmd}: {e}")))
}

fn validate_prefix(prefix: &str) -> Result<String, PluginError> {
    let trimmed = prefix.trim();
    if trimmed.is_empty() || trimmed.len() > 24 {
        return Err(invalid("prefix", "prefix must be 1 to 24 characters"));
    }
    let mut chars = trimmed.chars();
    let first = chars.next().ok_or_else(|| invalid("prefix", "prefix cannot be empty"))?;
    if !first.is_ascii_alphabetic() {
        return Err(invalid("prefix", "prefix must start with an ASCII letter"));
    }
    if !chars.all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return Err(invalid(
            "prefix",
            "prefix may contain only ASCII letters, digits, and underscores",
        ));
    }
    Ok(trimmed.to_string())
}

fn list_existing_params() -> Result<BTreeSet<String>, PluginError> {
    let res = run_cmd("parameters.list", json!({}))?;
    let mut set = BTreeSet::new();
    if let Some(arr) = res.get("parameters").and_then(Value::as_array) {
        for item in arr {
            if let Some(name) = item.get("name").and_then(Value::as_str) {
                set.insert(name.to_string());
            }
        }
    }
    Ok(set)
}

fn upsert_param(
    existing: &mut BTreeSet<String>,
    name: &str,
    expression: &str,
    unit: &str,
    comment: &str,
    favorite: bool,
) -> Result<(), PluginError> {
    if existing.contains(name) {
        run_cmd(
            "parameters.change",
            json!({
                "name": name,
                "expression": expression,
                "unit": unit,
            }),
        )?;
    } else {
        run_cmd(
            "parameters.add",
            json!({
                "name": name,
                "expression": expression,
                "unit": unit,
                "comment": comment,
                "favorite": favorite,
            }),
        )?;
        existing.insert(name.to_string());
    }
    Ok(())
}

fn fmt_num(v: f64) -> String {
    if libm::fabs(v - libm::round(v)) < 1e-9 {
        format!("{}", libm::round(v) as i64)
    } else {
        let s = format!("{v:.6}");
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    }
}

struct GearSpec<'a> {
    derived_prefix: &'a str,
    m_param: &'a str,
    z_param: &'a str,
    alpha_param: &'a str,
    fw_param: &'a str,
    x_param: &'a str,
    bl_param: &'a str,
    bore_param: Option<&'a str>,
    phi_param: Option<&'a str>,
    origin_x_expr: Option<&'a str>,
    sketch_name: &'a str,
    body_name: &'a str,
    module_val: f64,
    teeth_val: u32,
    shift_val: f64,
    bore_val: f64,
}

fn extract_curve_endpoints(res: &Value, label: &str) -> Result<(String, String), PluginError> {
    let pts = res
        .get("created")
        .and_then(|c| c.get("points"))
        .and_then(Value::as_array)
        .ok_or_else(|| PluginError::Trap(format!("{label}: missing created.points")))?;
    let first = pts
        .first()
        .and_then(Value::as_str)
        .ok_or_else(|| PluginError::Trap(format!("{label}: empty created.points")))?;
    let last = pts
        .last()
        .and_then(Value::as_str)
        .ok_or_else(|| PluginError::Trap(format!("{label}: missing end point")))?;
    Ok((first.to_string(), last.to_string()))
}

fn build_parametric_gear(
    existing: &mut BTreeSet<String>,
    spec: &GearSpec<'_>,
) -> Result<Value, PluginError> {
    let gp = spec.derived_prefix;
    let m = spec.m_param;
    let z = spec.z_param;
    let alpha = spec.alpha_param;
    let fw = spec.fw_param;
    let x = spec.x_param;
    let bl = spec.bl_param;

    let rp = format!("{gp}_rp");
    let rb = format!("{gp}_rb");
    let ra = format!("{gp}_ra");
    let rf = format!("{gp}_rf");
    let r_start = format!("{gp}_r_start");
    let t_start = format!("{gp}_t_start");
    let t_end = format!("{gp}_t_end");
    let inv_a = format!("{gp}_inv_a");
    let psi = format!("{gp}_psi");

    // Validate basic geometry against numeric values before issuing CAD commands.
    let root_dia = spec.module_val * (spec.teeth_val as f64 - 2.5 + 2.0 * spec.shift_val);
    if root_dia <= 0.5 {
        return Err(invalid("teeth", "root diameter is too small for this module/shift"));
    }
    if spec.bore_val > 0.0 && spec.bore_val >= root_dia - 0.5 {
        return Err(invalid(
            "bore_diameter",
            format!("bore ({:.2} mm) must be smaller than root diameter ({:.2} mm)", spec.bore_val, root_dia),
        ));
    }

    // Derived involute geometry parameters (auto-rebuilt by SolidCraft kernel).
    upsert_param(existing, &rp, &format!("{m} * {z} / 2"), "mm", "Pitch radius", false)?;
    upsert_param(existing, &rb, &format!("{rp} * cos({alpha})"), "mm", "Base radius", false)?;
    upsert_param(existing, &ra, &format!("{m} * ({z} / 2 + 1 + {x})"), "mm", "Tip (addendum) radius", false)?;
    upsert_param(existing, &rf, &format!("{m} * ({z} / 2 - 1.25 + {x})"), "mm", "Root (dedendum) radius", false)?;
    upsert_param(existing, &r_start, &format!("max({rb}, {rf})"), "mm", "Involute start radius", false)?;
    upsert_param(
        existing,
        &t_start,
        &format!("sqrt(max(0, ({r_start} / {rb}) * ({r_start} / {rb}) - 1))"),
        "",
        "Involute start roll parameter (rad)",
        false,
    )?;
    upsert_param(
        existing,
        &t_end,
        &format!("sqrt(max(0.0001, ({ra} / {rb}) * ({ra} / {rb}) - 1))"),
        "",
        "Involute tip roll parameter (rad)",
        false,
    )?;
    upsert_param(
        existing,
        &inv_a,
        &format!("tan({alpha}) - {alpha} / 1 rad"),
        "",
        "Involute function at pressure angle",
        false,
    )?;
    upsert_param(
        existing,
        &psi,
        &format!("(pi / 2 + 2 * {x} * tan({alpha}) - {bl} / {m}) / {z} + {inv_a}"),
        "",
        "Half tooth angular width on base circle (rad)",
        false,
    )?;

    let base_origin = match spec.origin_x_expr {
        Some(ox) => json!([ox, 0.0, 0.0]),
        None => json!([0.0, 0.0, 0.0]),
    };

    // 1. Root cylinder hub (parametric radius & height & center).
    let hub_res = run_cmd(
        "solid.cylinder",
        json!({
            "radius": rf,
            "height": fw,
            "base": base_origin,
            "operation": "new_body",
            "body_name": spec.body_name,
        }),
    )?;
    let actual_body = hub_res
        .get("created_bodies")
        .and_then(Value::as_array)
        .and_then(|a| a.first())
        .and_then(Value::as_str)
        .unwrap_or(spec.body_name)
        .to_string();

    // 2. Parametric single-tooth sketch centered on gear axis.
    let plane_arg = match spec.origin_x_expr {
        Some(ox) => json!({
            "origin": [ox, 0.0, 0.0],
            "x_dir": [1.0, 0.0, 0.0],
            "y_dir": [0.0, 1.0, 0.0],
        }),
        None => json!("XY"),
    };
    run_cmd(
        "sketch.create",
        json!({
            "plane": plane_arg,
            "name": spec.sketch_name,
        }),
    )?;

    let (x_bot, y_bot, x_top, y_top, tip_mid) = match spec.phi_param {
        Some(phi) => (
            format!("{rb} * (cos({phi} - {psi} + t) + t * sin({phi} - {psi} + t))"),
            format!("{rb} * (sin({phi} - {psi} + t) - t * cos({phi} - {psi} + t))"),
            format!("{rb} * (cos({phi} + {psi} - t) - t * sin({phi} + {psi} - t))"),
            format!("{rb} * (sin({phi} + {psi} - t) + t * cos({phi} + {psi} - t))"),
            json!([format!("{ra} * cos({phi})"), format!("{ra} * sin({phi})")]),
        ),
        None => (
            format!("{rb} * (cos(t - {psi}) + t * sin(t - {psi}))"),
            format!("{rb} * (sin(t - {psi}) - t * cos(t - {psi}))"),
            format!("{rb} * (cos({psi} - t) - t * sin({psi} - t))"),
            format!("{rb} * (sin({psi} - t) + t * cos({psi} - t))"),
            json!([ra, 0.0]),
        ),
    };

    let bot_res = run_cmd(
        "sketch.equation_curve",
        json!({
            "x": x_bot,
            "y": y_bot,
            "start": t_start,
            "end": t_end,
            "tolerance": 0.01,
        }),
    )?;
    let (bot_start, bot_end) = extract_curve_endpoints(&bot_res, "bottom flank")?;

    let top_res = run_cmd(
        "sketch.equation_curve",
        json!({
            "x": x_top,
            "y": y_top,
            "start": t_start,
            "end": t_end,
            "tolerance": 0.01,
        }),
    )?;
    let (top_start, top_end) = extract_curve_endpoints(&top_res, "top flank")?;

    // Close wedge at center [0, 0] (always inside root cylinder) and along tip arc.
    run_cmd(
        "sketch.line",
        json!({
            "points": [[0.0, 0.0], bot_start],
            "connect": true,
        }),
    )?;
    run_cmd(
        "sketch.line",
        json!({
            "points": [[0.0, 0.0], top_start],
            "connect": true,
        }),
    )?;
    run_cmd(
        "sketch.arc.three_point",
        json!({
            "start": bot_end,
            "through": tip_mid,
            "end": top_end,
            "connect": true,
        }),
    )?;
    run_cmd("sketch.finish", json!({}))?;

    // 3. Extrude single tooth wedge and join onto root hub.
    let tooth_name = format!("{gp}_Tooth");
    let ext_res = run_cmd(
        "solid.extrude",
        json!({
            "sketch": spec.sketch_name,
            "distance": fw,
            "operation": "join",
            "targets": [&actual_body],
            "name": tooth_name,
        }),
    )?;
    let tooth_feature = ext_res
        .get("name")
        .and_then(Value::as_str)
        .unwrap_or(&tooth_name)
        .to_string();

    // 4. Parametric circular pattern of the tooth around gear axis.
    run_cmd(
        "solid.pattern.circular",
        json!({
            "features": [tooth_feature],
            "axis": {
                "origin": base_origin,
                "dir": [0.0, 0.0, 1.0],
            },
            "count": z,
            "angle": "360 deg",
        }),
    )?;

    // 5. Optional parametric bore cut.
    if let Some(bore_p) = spec.bore_param {
        if spec.bore_val > 0.0 {
            run_cmd(
                "solid.cylinder",
                json!({
                    "diameter": bore_p,
                    "height": fw,
                    "base": base_origin,
                    "operation": "cut",
                    "targets": [&actual_body],
                }),
            )?;
        }
    }

    let pitch_dia = spec.module_val * spec.teeth_val as f64;
    let outer_dia = spec.module_val * (spec.teeth_val as f64 + 2.0 + 2.0 * spec.shift_val);
    Ok(json!({
        "body": actual_body,
        "sketch": spec.sketch_name,
        "pitch_diameter_mm": pitch_dia,
        "outer_diameter_mm": outer_dia,
        "root_diameter_mm": root_dia,
    }))
}

#[derive(Deserialize)]
#[serde(default)]
struct SpurGearInput {
    prefix: String,
    module: f64,
    teeth: u32,
    pressure_angle: f64,
    face_width: f64,
    bore_diameter: f64,
    profile_shift: f64,
    backlash: f64,
    body_name: String,
}

impl Default for SpurGearInput {
    fn default() -> Self {
        Self {
            prefix: "g1".into(),
            module: 2.0,
            teeth: 20,
            pressure_angle: 20.0,
            face_width: 10.0,
            bore_diameter: 8.0,
            profile_shift: 0.0,
            backlash: 0.05,
            body_name: "SpurGear".into(),
        }
    }
}

fn run_spur_gear(raw: &Value) -> Result<String, PluginError> {
    let inp: SpurGearInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    if inp.module <= 0.0 || inp.face_width <= 0.0 {
        return Err(invalid("module", "module and face_width must be positive"));
    }
    if inp.teeth < 8 || inp.teeth > 80 {
        return Err(invalid("teeth", "teeth must be between 8 and 80"));
    }

    let mut existing = list_existing_params()?;

    let m_p = format!("{p}_m");
    let z_p = format!("{p}_z");
    let a_p = format!("{p}_alpha");
    let fw_p = format!("{p}_fw");
    let x_p = format!("{p}_x");
    let bl_p = format!("{p}_bl");
    let bore_p = format!("{p}_bore");

    upsert_param(&mut existing, &m_p, &fmt_num(inp.module), "mm", "Gear module", true)?;
    upsert_param(&mut existing, &z_p, &format!("{}", inp.teeth), "", "Tooth count", true)?;
    upsert_param(&mut existing, &a_p, &fmt_num(inp.pressure_angle), "deg", "Pressure angle", true)?;
    upsert_param(&mut existing, &fw_p, &fmt_num(inp.face_width), "mm", "Face width", true)?;
    upsert_param(&mut existing, &x_p, &fmt_num(inp.profile_shift), "", "Profile shift coefficient", false)?;
    upsert_param(&mut existing, &bl_p, &fmt_num(inp.backlash), "mm", "Backlash", false)?;
    if inp.bore_diameter > 0.0 {
        upsert_param(&mut existing, &bore_p, &fmt_num(inp.bore_diameter), "mm", "Bore diameter", true)?;
    }

    let sketch_name = format!("{p}_ToothSketch");
    let body_name = if inp.body_name.trim().is_empty() {
        format!("{p}_Gear")
    } else {
        inp.body_name.trim().to_string()
    };

    let info = build_parametric_gear(
        &mut existing,
        &GearSpec {
            derived_prefix: &p,
            m_param: &m_p,
            z_param: &z_p,
            alpha_param: &a_p,
            fw_param: &fw_p,
            x_param: &x_p,
            bl_param: &bl_p,
            bore_param: if inp.bore_diameter > 0.0 { Some(&bore_p) } else { None },
            phi_param: None,
            origin_x_expr: None,
            sketch_name: &sketch_name,
            body_name: &body_name,
            module_val: inp.module,
            teeth_val: inp.teeth,
            shift_val: inp.profile_shift,
            bore_val: inp.bore_diameter,
        },
    )?;

    let out = json!({
        "mode": "parametric-spur-gear",
        "prefix": p,
        "gear": info,
        "editable_parameters": [m_p, z_p, a_p, fw_p, x_p, bl_p],
    });
    Ok(out.to_string())
}

#[derive(Deserialize)]
#[serde(default)]
struct ParallelPairInput {
    prefix: String,
    module: f64,
    teeth_1: u32,
    teeth_2: u32,
    pressure_angle: f64,
    face_width: f64,
    bore_1: f64,
    bore_2: f64,
    backlash: f64,
}

impl Default for ParallelPairInput {
    fn default() -> Self {
        Self {
            prefix: "p1".into(),
            module: 2.0,
            teeth_1: 16,
            teeth_2: 32,
            pressure_angle: 20.0,
            face_width: 10.0,
            bore_1: 6.0,
            bore_2: 10.0,
            backlash: 0.05,
        }
    }
}

fn run_parallel_pair(raw: &Value) -> Result<String, PluginError> {
    let inp: ParallelPairInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    if inp.module <= 0.0 || inp.face_width <= 0.0 {
        return Err(invalid("module", "module and face_width must be positive"));
    }
    if inp.teeth_1 < 8 || inp.teeth_1 > 80 || inp.teeth_2 < 8 || inp.teeth_2 > 80 {
        return Err(invalid("teeth_1", "tooth counts must be between 8 and 80"));
    }

    let mut existing = list_existing_params()?;

    // Shared pair parameters
    let m_p = format!("{p}_m");
    let z1_p = format!("{p}_z1");
    let z2_p = format!("{p}_z2");
    let a_p = format!("{p}_alpha");
    let fw_p = format!("{p}_fw");
    let x0_p = format!("{p}_x");
    let bl_p = format!("{p}_bl");
    let b1_p = format!("{p}_bore1");
    let b2_p = format!("{p}_bore2");
    let cd_p = format!("{p}_cd");
    let ratio_p = format!("{p}_ratio");
    let phi2_p = format!("{p}_g2_phi");

    upsert_param(&mut existing, &m_p, &fmt_num(inp.module), "mm", "Shared module", true)?;
    upsert_param(&mut existing, &z1_p, &format!("{}", inp.teeth_1), "", "Pinion tooth count (Z1)", true)?;
    upsert_param(&mut existing, &z2_p, &format!("{}", inp.teeth_2), "", "Gear tooth count (Z2)", true)?;
    upsert_param(&mut existing, &a_p, &fmt_num(inp.pressure_angle), "deg", "Shared pressure angle", true)?;
    upsert_param(&mut existing, &fw_p, &fmt_num(inp.face_width), "mm", "Shared face width", true)?;
    upsert_param(&mut existing, &x0_p, "0", "", "Profile shift coefficient", false)?;
    upsert_param(&mut existing, &bl_p, &fmt_num(inp.backlash), "mm", "Backlash per gear", false)?;
    if inp.bore_1 > 0.0 {
        upsert_param(&mut existing, &b1_p, &fmt_num(inp.bore_1), "mm", "Pinion bore diameter", true)?;
    }
    if inp.bore_2 > 0.0 {
        upsert_param(&mut existing, &b2_p, &fmt_num(inp.bore_2), "mm", "Gear bore diameter", true)?;
    }

    // Auto-rebuilding center distance, gear ratio, and Gear 2 meshing phase!
    upsert_param(
        &mut existing,
        &cd_p,
        &format!("{m_p} * ({z1_p} + {z2_p}) / 2"),
        "mm",
        "Shaft center distance (auto-updates with m, Z1, Z2)",
        true,
    )?;
    upsert_param(
        &mut existing,
        &ratio_p,
        &format!("{z2_p} / {z1_p}"),
        "",
        "Gear ratio Z2 / Z1",
        true,
    )?;
    upsert_param(
        &mut existing,
        &phi2_p,
        &format!("pi + pi / {z2_p}"),
        "",
        "Gear 2 meshing phase angle (rad)",
        false,
    )?;

    let g1_prefix = format!("{p}_g1");
    let g1_sketch = format!("{p}_PinionTooth");
    let g1_body = format!("{p}_Pinion");
    let g1 = build_parametric_gear(
        &mut existing,
        &GearSpec {
            derived_prefix: &g1_prefix,
            m_param: &m_p,
            z_param: &z1_p,
            alpha_param: &a_p,
            fw_param: &fw_p,
            x_param: &x0_p,
            bl_param: &bl_p,
            bore_param: if inp.bore_1 > 0.0 { Some(&b1_p) } else { None },
            phi_param: None,
            origin_x_expr: None,
            sketch_name: &g1_sketch,
            body_name: &g1_body,
            module_val: inp.module,
            teeth_val: inp.teeth_1,
            shift_val: 0.0,
            bore_val: inp.bore_1,
        },
    )?;

    let g2_prefix = format!("{p}_g2");
    let g2_sketch = format!("{p}_GearTooth");
    let g2_body = format!("{p}_Gear");
    let g2 = build_parametric_gear(
        &mut existing,
        &GearSpec {
            derived_prefix: &g2_prefix,
            m_param: &m_p,
            z_param: &z2_p,
            alpha_param: &a_p,
            fw_param: &fw_p,
            x_param: &x0_p,
            bl_param: &bl_p,
            bore_param: if inp.bore_2 > 0.0 { Some(&b2_p) } else { None },
            phi_param: Some(&phi2_p),
            origin_x_expr: Some(&cd_p),
            sketch_name: &g2_sketch,
            body_name: &g2_body,
            module_val: inp.module,
            teeth_val: inp.teeth_2,
            shift_val: 0.0,
            bore_val: inp.bore_2,
        },
    )?;

    let cd_mm = inp.module * (inp.teeth_1 + inp.teeth_2) as f64 / 2.0;
    let ratio = inp.teeth_2 as f64 / inp.teeth_1 as f64;
    let out = json!({
        "mode": "parametric-parallel-pair",
        "prefix": p,
        "center_distance_mm": cd_mm,
        "gear_ratio": ratio,
        "pinion": g1,
        "gear": g2,
        "editable_parameters": [m_p, z1_p, z2_p, a_p, fw_p, bl_p, cd_p, ratio_p],
    });
    Ok(out.to_string())
}

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use serde::Deserialize;
use serde_json::{Value, json};

use crate::cylindrical::{GearSide, GearSpec, ToothKind, build_parametric_gear};
use crate::placement::{PlacementFrame, resolve_placement};
use crate::{
    PluginError, choose_unique_prefix, extract_created_curve_id, fmt_num, invalid,
    list_existing_doc_names, list_existing_params, pick_outer_profile_index, run_cmd, unique_name,
    upsert_param, validate_prefix,
};

#[derive(Deserialize)]
#[serde(default)]
struct WormDriveInput {
    placement: Vec<Value>,
    axis_name: String,
    base_plane: String,
    layout_angle: f64,
    flip_direction: bool,
    axial_offset: f64,
    prefix: String,
    module: f64,
    worm_starts: u32,
    wheel_teeth: u32,
    worm_pitch_dia: f64,
    worm_length: f64,
    pressure_angle: f64,
    wheel_width: f64,
    worm_bore: f64,
    wheel_bore: f64,
    hand: String,
    backlash: f64,
}

impl Default for WormDriveInput {
    fn default() -> Self {
        Self {
            placement: Vec::new(),
            axis_name: String::new(),
            base_plane: "XY".into(),
            layout_angle: 0.0,
            flip_direction: false,
            axial_offset: 0.0,
            prefix: "wm1".into(),
            module: 2.0,
            worm_starts: 1,
            wheel_teeth: 24,
            worm_pitch_dia: 20.0,
            worm_length: 22.0,
            pressure_angle: 20.0,
            wheel_width: 12.0,
            worm_bore: 8.0,
            wheel_bore: 10.0,
            hand: "right".into(),
            backlash: 0.08,
        }
    }
}

pub fn run_worm_drive(raw: &Value) -> Result<String, PluginError> {
    let inp: WormDriveInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    if inp.module <= 0.0 || inp.worm_length <= 1.0 || inp.wheel_width <= 1.0 {
        return Err(invalid("module", "module, worm_length, and wheel_width must be positive"));
    }
    if inp.worm_starts < 1 || inp.worm_starts > 4 {
        return Err(invalid("worm_starts", "worm_starts must be between 1 and 4"));
    }
    if inp.wheel_teeth < 12 || inp.wheel_teeth > 80 {
        return Err(invalid("wheel_teeth", "wheel_teeth must be between 12 and 80"));
    }
    let rf_worm = inp.worm_pitch_dia / 2.0 - 1.25 * inp.module;
    let ra_worm = inp.worm_pitch_dia / 2.0 + inp.module;
    if rf_worm <= 1.0 {
        return Err(invalid("worm_pitch_dia", "worm_pitch_dia is too small for this module"));
    }
    if inp.worm_bore > 0.0 && inp.worm_bore >= 2.0 * rf_worm - 0.5 {
        return Err(invalid("worm_bore", "worm_bore must be smaller than worm root diameter"));
    }

    let frame = resolve_placement(
        &inp.placement,
        &inp.axis_name,
        &inp.base_plane,
        inp.flip_direction,
        inp.axial_offset,
        inp.layout_angle,
    )?;
    let mut existing = list_existing_params()?;
    let mut existing_names = list_existing_doc_names()?;
    let p = choose_unique_prefix(&p, &existing, &existing_names);

    let z1 = inp.worm_starts as f64;
    let z2 = inp.wheel_teeth as f64;
    let lead = core::f64::consts::PI * inp.module * z1;
    let lead_angle_rad = libm::atan((inp.module * z1) / inp.worm_pitch_dia);
    let lead_angle_deg = lead_angle_rad * 180.0 / core::f64::consts::PI;
    let hand_sign = if inp.hand == "left" { -1.0 } else { 1.0 };

    // Clamp single-sweep turns to <= 3.75 (host sweep limit is 4 turns).
    let max_len = 3.75 * lead;
    let actual_worm_len = if inp.worm_length > max_len { max_len } else { inp.worm_length };
    let twist_deg = hand_sign * (actual_worm_len / lead) * 360.0;

    let m_p = format!("{p}_m");
    let z2_p = format!("{p}_z2");
    let a_p = format!("{p}_alpha");
    let beta_p = format!("{p}_lead_ang");
    let fw_p = format!("{p}_wheel_w");
    let x0_p = format!("{p}_x");
    let bl_p = format!("{p}_bl");
    let wbore_p = format!("{p}_wheel_bore");
    let cd_p = format!("{p}_cd");

    upsert_param(&mut existing, &m_p, &fmt_num(inp.module), "mm", "Axial module", true)?;
    upsert_param(&mut existing, &z2_p, &format!("{}", inp.wheel_teeth), "", "Worm wheel teeth", true)?;
    upsert_param(&mut existing, &a_p, &fmt_num(inp.pressure_angle), "deg", "Pressure angle", true)?;
    upsert_param(&mut existing, &beta_p, &fmt_num(lead_angle_deg), "deg", "Worm lead angle gamma", true)?;
    upsert_param(&mut existing, &fw_p, &fmt_num(inp.wheel_width), "mm", "Worm wheel face width", true)?;
    upsert_param(&mut existing, &x0_p, "0", "", "Profile shift", false)?;
    upsert_param(&mut existing, &bl_p, &fmt_num(inp.backlash), "mm", "Backlash", false)?;
    if inp.wheel_bore > 0.0 {
        upsert_param(&mut existing, &wbore_p, &fmt_num(inp.wheel_bore), "mm", "Wheel bore diameter", true)?;
    }
    let cd_val = (inp.worm_pitch_dia + inp.module * z2) / 2.0;
    upsert_param(&mut existing, &cd_p, &fmt_num(cd_val), "mm", "Worm drive center distance", true)?;

    // 1. Build Worm along local Y axis at x = 0, z = wheel_width/2, spanning y in [-L/2, +L/2].
    // Worm local frame: z_dir = frame.y_dir, x_dir = frame.x_dir, y_dir = -frame.z_dir.
    let worm_origin = frame.world_point(0.0, -actual_worm_len / 2.0, inp.wheel_width / 2.0);
    let worm_frame = PlacementFrame {
        origin: worm_origin,
        x_dir: frame.x_dir,
        y_dir: [-frame.z_dir[0], -frame.z_dir[1], -frame.z_dir[2]],
        z_dir: frame.y_dir,
    };

    let worm_sk = unique_name(&format!("{p}_WormProfile"), &mut existing_names);
    let worm_path = unique_name(&format!("{p}_WormAxis"), &mut existing_names);
    let worm_body = unique_name(&format!("{p}_Worm"), &mut existing_names);

    run_cmd(
        "sketch.create",
        json!({ "plane": worm_frame.plane_json(json!(worm_origin)), "name": worm_sk }),
    )?;
    if inp.worm_bore > 0.0 {
        run_cmd("sketch.circle.center", json!({ "center": [0.0, 0.0], "diameter": inp.worm_bore }))?;
    }

    // Build closed z1-start trapezoidal thread cross-section around [0, 0].
    let n_starts = inp.worm_starts as usize;
    let sector = 2.0 * core::f64::consts::PI / (n_starts as f64);
    let mut pts: Vec<Value> = Vec::new();
    for k in 0..n_starts {
        let base_a = (k as f64) * sector;
        // Root arc / lobe profile per start (8 points per start -> smooth & watertight)
        let a0 = base_a - 0.35 * sector;
        let a1 = base_a - 0.22 * sector;
        let a2 = base_a - 0.12 * sector;
        let a3 = base_a + 0.12 * sector;
        let a4 = base_a + 0.22 * sector;
        let a5 = base_a + 0.35 * sector;
        let a6 = base_a + 0.50 * sector;
        pts.push(json!([rf_worm * libm::cos(a0), rf_worm * libm::sin(a0)]));
        pts.push(json!([rf_worm * libm::cos(a1), rf_worm * libm::sin(a1)]));
        pts.push(json!([ra_worm * libm::cos(a2), ra_worm * libm::sin(a2)]));
        pts.push(json!([ra_worm * libm::cos(a3), ra_worm * libm::sin(a3)]));
        pts.push(json!([rf_worm * libm::cos(a4), rf_worm * libm::sin(a4)]));
        pts.push(json!([rf_worm * libm::cos(a5), rf_worm * libm::sin(a5)]));
        pts.push(json!([rf_worm * libm::cos(a6), rf_worm * libm::sin(a6)]));
    }
    run_cmd("sketch.line", json!({ "points": pts, "closed": true }))?;
    run_cmd("sketch.finish", json!({}))?;
    let worm_prof = pick_outer_profile_index(&worm_sk, false)?;

    run_cmd(
        "sketch.create",
        json!({
            "plane": {
                "origin": worm_origin,
                "x_dir": [worm_frame.x_dir[0], worm_frame.x_dir[1], worm_frame.x_dir[2]],
                "y_dir": [worm_frame.z_dir[0], worm_frame.z_dir[1], worm_frame.z_dir[2]],
            },
            "name": worm_path,
        }),
    )?;
    let lres = run_cmd("sketch.line", json!({ "points": [[0.0, 0.0], [0.0, actual_worm_len]] }))?;
    let pcurve = extract_created_curve_id(&lres, "worm axis")?;
    run_cmd("sketch.finish", json!({}))?;

    run_cmd(
        "solid.sweep",
        json!({
            "sketch": worm_sk,
            "profiles": [worm_prof],
            "path_sketch": worm_path,
            "path": [pcurve],
            "twist": format!("{} deg", fmt_num(twist_deg)),
            "operation": "new_body",
            "body_name": worm_body,
        }),
    )?;

    // 2. Build mating Helical Worm Wheel at x = cd_p, axis along frame.z_dir, helix angle = lead_angle!
    let wheel_sk = unique_name(&format!("{p}_WheelSketch"), &mut existing_names);
    let wheel_path = unique_name(&format!("{p}_WheelAxis"), &mut existing_names);
    let wheel_body = unique_name(&format!("{p}_WormWheel"), &mut existing_names);
    let wheel = build_parametric_gear(
        &mut existing,
        &GearSpec {
            derived_prefix: &format!("{p}_wh"),
            m_param: &m_p,
            mt_param: &m_p,
            at_param: &a_p,
            z_param: &z2_p,
            fw_param: &fw_p,
            x_param: &x0_p,
            bl_param: &bl_p,
            bore_param: if inp.wheel_bore > 0.0 { Some(&wbore_p) } else { None },
            rim_param: None,
            phi_param: None,
            gear_side: GearSide::External,
            tooth_kind: ToothKind::Helical,
            beta_param: Some(&beta_p),
            hand_sign,
            frame,
            offset_x_param: Some(&cd_p),
            sketch_name: &wheel_sk,
            path_sketch_name: &wheel_path,
            body_name: &wheel_body,
            m_val: inp.module,
            mt_val: inp.module,
            at_deg: inp.pressure_angle,
            teeth_val: inp.wheel_teeth,
            shift_val: 0.0,
            bore_val: inp.wheel_bore,
            rim_val: 0.0,
        },
    )?;

    Ok(json!({
        "mode": "gm-worm-drive",
        "prefix": p,
        "ratio": z2 / z1,
        "lead_angle_deg": lead_angle_deg,
        "center_distance_mm": cd_val,
        "worm_body": worm_body,
        "wheel": wheel,
    })
    .to_string())
}

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use serde::Deserialize;
use serde_json::{Value, json};

use crate::cylindrical::{GearSide, GearSpec, ToothKind, build_parametric_gear};
use crate::placement::resolve_placement;
use crate::{
    PluginError, choose_unique_prefix, fmt_num, invalid, list_existing_doc_names,
    list_existing_params, run_cmd, unique_name, upsert_param, validate_prefix,
};

#[derive(Deserialize)]
#[serde(default)]
struct RackPinionInput {
    placement: Vec<Value>,
    axis_name: String,
    base_plane: String,
    layout_angle: f64,
    flip_direction: bool,
    axial_offset: f64,
    prefix: String,
    module: f64,
    pinion_teeth: u32,
    rack_sizing: String,
    rack_teeth: u32,
    rack_length: f64,
    pressure_angle: f64,
    face_width: f64,
    rack_backing: f64,
    pinion_bore: f64,
    backlash: f64,
    create_joints: bool,
}

impl Default for RackPinionInput {
    fn default() -> Self {
        Self {
            placement: Vec::new(),
            axis_name: String::new(),
            base_plane: "XY".into(),
            layout_angle: 0.0,
            flip_direction: false,
            axial_offset: 0.0,
            prefix: "rp1".into(),
            module: 2.0,
            pinion_teeth: 16,
            rack_sizing: "by_teeth".into(),
            rack_teeth: 12,
            rack_length: 80.0,
            pressure_angle: 20.0,
            face_width: 10.0,
            rack_backing: 8.0,
            pinion_bore: 6.0,
            backlash: 0.05,
            create_joints: false,
        }
    }
}

pub fn run_rack_pinion(raw: &Value) -> Result<String, PluginError> {
    let mut inp: RackPinionInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    if inp.module <= 0.0 || inp.face_width <= 0.0 || inp.rack_backing <= 0.5 {
        return Err(invalid("module", "module, face_width, and rack_backing must be positive"));
    }
    if inp.pinion_teeth < 8 || inp.pinion_teeth > 80 {
        return Err(invalid("pinion_teeth", "pinion_teeth must be between 8 and 80"));
    }
    let pitch_mm = core::f64::consts::PI * inp.module;
    let target_len = if inp.rack_sizing == "by_length" {
        if inp.rack_length < 3.0 * pitch_mm {
            return Err(invalid("rack_length", format!("rack_length must be at least {:.1} mm (3 pitches)", 3.0 * pitch_mm)));
        }
        inp.rack_teeth = (libm::floor(inp.rack_length / pitch_mm) as u32).clamp(3, 120);
        inp.rack_length
    } else {
        if inp.rack_teeth < 3 || inp.rack_teeth > 120 {
            return Err(invalid("rack_teeth", "rack_teeth must be between 3 and 120"));
        }
        (inp.rack_teeth as f64) * pitch_mm
    };

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

    let m_p = format!("{p}_m");
    let z_p = format!("{p}_z");
    let a_p = format!("{p}_alpha");
    let fw_p = format!("{p}_fw");
    let x0_p = format!("{p}_x");
    let bl_p = format!("{p}_bl");
    let bl_half_p = format!("{p}_bl_half");
    let bore_p = format!("{p}_bore");

    upsert_param(&mut existing, &m_p, &fmt_num(inp.module), "mm", "Module", true)?;
    upsert_param(&mut existing, &z_p, &format!("{}", inp.pinion_teeth), "", "Pinion teeth", true)?;
    upsert_param(&mut existing, &a_p, &fmt_num(inp.pressure_angle), "deg", "Pressure angle", true)?;
    upsert_param(&mut existing, &fw_p, &fmt_num(inp.face_width), "mm", "Face width", true)?;
    upsert_param(&mut existing, &x0_p, "0", "", "Profile shift", false)?;
    upsert_param(&mut existing, &bl_p, &fmt_num(inp.backlash), "mm", "Total backlash", true)?;
    upsert_param(&mut existing, &bl_half_p, &format!("{bl_p} / 2"), "mm", "Half backlash", false)?;
    if inp.pinion_bore > 0.0 {
        upsert_param(&mut existing, &bore_p, &fmt_num(inp.pinion_bore), "mm", "Pinion bore", true)?;
    }

    // 1. Build Pinion at origin
    let pin_sk = unique_name(&format!("{p}_PinionSketch"), &mut existing_names);
    let pin_path = unique_name(&format!("{p}_PinionAxis"), &mut existing_names);
    let pin_body = unique_name(&format!("{p}_Pinion"), &mut existing_names);
    let pinion = build_parametric_gear(
        &mut existing,
        &GearSpec {
            derived_prefix: &format!("{p}_pin"),
            m_param: &m_p,
            mt_param: &m_p,
            at_param: &a_p,
            z_param: &z_p,
            fw_param: &fw_p,
            x_param: &x0_p,
            bl_param: &bl_half_p,
            bore_param: if inp.pinion_bore > 0.0 { Some(&bore_p) } else { None },
            rim_param: None,
            phi_param: None,
            gear_side: GearSide::External,
            tooth_kind: ToothKind::Straight,
            beta_param: None,
            hand_sign: 1.0,
            frame,
            offset_x_param: None,
            sketch_name: &pin_sk,
            path_sketch_name: &pin_path,
            body_name: &pin_body,
            m_val: inp.module,
            mt_val: inp.module,
            at_deg: inp.pressure_angle,
            teeth_val: inp.pinion_teeth,
            shift_val: 0.0,
            bore_val: inp.pinion_bore,
            rim_val: 0.0,
            root_fillet: false,
            preview_only: crate::is_preview(raw),
        },
    )?;

    // 2. Build ISO 53 Linear Rack tangent to pinion pitch circle at x = rp = m * z / 2.
    // Pinion has a tooth centered on +X (y = 0), so Rack has a tooth *space* centered at y = 0!
    let m = inp.module;
    let pitch = core::f64::consts::PI * m;
    let rp = m * (inp.pinion_teeth as f64) / 2.0;
    let alpha_rad = inp.pressure_angle * core::f64::consts::PI / 180.0;
    let tan_a = libm::tan(alpha_rad);
    let add = m;
    let ded = 1.25 * m;
    let half_bl = inp.backlash / 2.0;
    let x_tip = rp - add;
    let x_root = rp + ded;
    let x_back = x_root + inp.rack_backing;

    // Half tooth width at pitch line = pitch/4 - half_bl/2.
    // Space centered at y=0, so teeth are centered at y = (i + 0.5) * pitch.
    let n = inp.rack_teeth as i32;
    let teeth_span = (n as f64) * pitch;
    let y_tooth_start = -teeth_span / 2.0;
    let y_start = -target_len / 2.0;
    let y_end = target_len / 2.0;
    let hw_pitch = pitch / 4.0 - half_bl / 2.0;
    let hw_tip = (hw_pitch - add * tan_a).max(0.05 * m);
    let hw_root = hw_pitch + ded * tan_a;

    let mut pts: Vec<Value> = Vec::new();
    pts.push(json!([x_back, y_start]));
    pts.push(json!([x_root, y_start]));
    for i in 0..n {
        let yc = y_tooth_start + (i as f64 + 0.5) * pitch;
        pts.push(json!([x_root, yc - hw_root]));
        pts.push(json!([x_tip, yc - hw_tip]));
        pts.push(json!([x_tip, yc + hw_tip]));
        pts.push(json!([x_root, yc + hw_root]));
    }
    pts.push(json!([x_root, y_end]));
    pts.push(json!([x_back, y_end]));

    let rack_sk = unique_name(&format!("{p}_RackSketch"), &mut existing_names);
    let rack_body = unique_name(&format!("{p}_Rack"), &mut existing_names);
    run_cmd(
        "sketch.create",
        json!({
            "plane": frame.plane_json(frame.origin_json(None)),
            "name": rack_sk,
        }),
    )?;
    run_cmd("sketch.line", json!({ "points": pts, "closed": true }))?;
    run_cmd("sketch.finish", json!({}))?;
    run_cmd(
        "solid.extrude",
        json!({
            "sketch": rack_sk,
            "distance": &fw_p,
            "operation": "new_body",
            "body_name": rack_body,
        }),
    )?;

    let mut joints_created = false;
    if inp.create_joints && !crate::is_preview(raw) {
        let pin_b = pinion
            .get("body")
            .and_then(Value::as_str)
            .unwrap_or(&pin_body)
            .to_string();
        let bodies = alloc::vec![pin_b.clone(), rack_body.clone()];
        crate::joints::make_components_from_bodies(&bodies)?;
        let j_pin = format!("{p}_PinionJoint");
        let j_rack = format!("{p}_RackJoint");
        let p_pin = crate::joints::offset_point(&frame, 0.0, 0.0, 0.0);
        let p_rack = crate::joints::offset_point(&frame, x_back, 0.0, 0.0);
        crate::joints::add_revolute_joint(&j_pin, &pin_b, p_pin, &frame)?;
        crate::joints::add_slider_joint(&j_rack, &rack_body, p_rack, frame.y_dir, frame.x_dir)?;
        crate::joints::add_rack_pinion_link(&j_pin, &j_rack, &format!("{p}_pin_rp"))?;
        joints_created = true;
    }

    Ok(json!({
        "mode": "gm-rack-pinion",
        "prefix": p,
        "pitch_mm": pitch,
        "rack_teeth": inp.rack_teeth,
        "rack_length_mm": target_len,
        "joints_created": joints_created,
        "pinion": pinion,
        "rack_body": rack_body,
    })
    .to_string())
}

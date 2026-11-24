use alloc::collections::BTreeSet;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use serde::Deserialize;
use serde_json::{Value, json};

use crate::placement::{PlacementFrame, resolve_placement};
use crate::{
    PluginError, choose_unique_prefix, extract_created_curve_id, extract_curve_endpoints, fmt_num,
    invalid, list_existing_doc_names, list_existing_params, pick_outer_profile_index, run_cmd,
    unique_name, upsert_param, validate_prefix,
};

#[derive(Deserialize)]
#[serde(default)]
struct BevelPairInput {
    placement: Vec<Value>,
    axis_name: String,
    base_plane: String,
    layout_angle: f64,
    flip_direction: bool,
    axial_offset: f64,
    prefix: String,
    bevel_type: String,
    spiral_angle: f64,
    hand: String,
    sizing_mode: String,
    module: f64,
    teeth_1: u32,
    teeth_2: u32,
    gear_ratio: f64,
    shaft_angle: f64,
    pressure_angle: f64,
    face_width: f64,
    bore_1: f64,
    bore_2: f64,
    backlash: f64,
}

impl Default for BevelPairInput {
    fn default() -> Self {
        Self {
            placement: Vec::new(),
            axis_name: String::new(),
            base_plane: "XY".into(),
            layout_angle: 0.0,
            flip_direction: false,
            axial_offset: 0.0,
            prefix: "bv1".into(),
            bevel_type: "straight".into(),
            spiral_angle: 30.0,
            hand: "right".into(),
            sizing_mode: "by_teeth".into(),
            module: 2.0,
            teeth_1: 16,
            teeth_2: 24,
            gear_ratio: 1.5,
            shaft_angle: 90.0,
            pressure_angle: 20.0,
            face_width: 10.0,
            bore_1: 6.0,
            bore_2: 8.0,
            backlash: 0.05,
        }
    }
}

fn draw_bevel_section(
    sketch_name: &str,
    frame: PlacementFrame,
    z_offset: f64,
    m_scaled: f64,
    teeth: u32,
    alpha_deg: f64,
    backlash_scaled: f64,
    phase_rad: f64,
) -> Result<u64, PluginError> {
    let origin = frame.world_point(0.0, 0.0, z_offset);
    run_cmd(
        "sketch.create",
        json!({ "plane": frame.plane_json(json!(origin)), "name": sketch_name }),
    )?;

    let alpha = alpha_deg * core::f64::consts::PI / 180.0;
    let rp = m_scaled * (teeth as f64) / 2.0;
    let rb = rp * libm::cos(alpha);
    let ra = rp + m_scaled;
    let rf = rp - 1.25 * m_scaled;
    let r_start = if rb > rf { rb } else { rf };
    let t_start = libm::sqrt(((r_start / rb) * (r_start / rb) - 1.0).max(0.0));
    let t_end = libm::sqrt(((ra / rb) * (ra / rb) - 1.0).max(0.0001));
    let inv_a = libm::tan(alpha) - alpha;
    let psi = (core::f64::consts::PI / 2.0 - backlash_scaled / m_scaled) / (teeth as f64) + inv_a;
    let hp = core::f64::consts::PI / (teeth as f64);

    let rb_s = fmt_num(rb);
    let psi_s = fmt_num(psi);
    let phi_s = fmt_num(phase_rad);
    let x_bot = format!("{rb_s} * (cos({phi_s} - {psi_s} + t) + t * sin({phi_s} - {psi_s} + t))");
    let y_bot = format!("{rb_s} * (sin({phi_s} - {psi_s} + t) - t * cos({phi_s} - {psi_s} + t))");
    let x_top = format!("{rb_s} * (cos({phi_s} + {psi_s} - t) - t * sin({phi_s} + {psi_s} - t))");
    let y_top = format!("{rb_s} * (sin({phi_s} + {psi_s} - t) + t * cos({phi_s} + {psi_s} - t))");

    let bot_res = run_cmd(
        "sketch.equation_curve",
        json!({ "x": x_bot, "y": y_bot, "start": t_start, "end": t_end, "tolerance": 0.02 }),
    )?;
    let bot_curve = extract_created_curve_id(&bot_res, "bevel bot flank")?;
    let (bot_start, bot_end) = extract_curve_endpoints(&bot_res, "bevel bot flank")?;

    let top_res = run_cmd(
        "sketch.equation_curve",
        json!({ "x": x_top, "y": y_top, "start": t_start, "end": t_end, "tolerance": 0.02 }),
    )?;
    let top_curve = extract_created_curve_id(&top_res, "bevel top flank")?;
    let (top_start, top_end) = extract_curve_endpoints(&top_res, "bevel top flank")?;

    let tip_mid = json!([ra * libm::cos(phase_rad), ra * libm::sin(phase_rad)]);
    let tip_res = run_cmd(
        "sketch.arc.three_point",
        json!({ "start": bot_end, "through": tip_mid, "end": top_end, "connect": true }),
    )?;
    let tip_curve = extract_created_curve_id(&tip_res, "bevel tip arc")?;

    let pt_rf = |a: f64| json!([rf * libm::cos(a), rf * libm::sin(a)]);
    let p_hp_neg = pt_rf(phase_rad - hp);
    let p_hp_pos = pt_rf(phase_rad + hp);

    let mut entities = Vec::new();
    entities.push(bot_curve);
    entities.push(top_curve);
    entities.push(tip_curve);

    if rf < rb - 1e-3 {
        let p_psi_neg = pt_rf(phase_rad - psi);
        let p_psi_pos = pt_rf(phase_rad + psi);
        let p_mid_neg = pt_rf(phase_rad - 0.5 * (hp + psi));
        let p_mid_pos = pt_rf(phase_rad + 0.5 * (hp + psi));
        let a_bot = run_cmd("sketch.arc.three_point", json!({ "start": p_hp_neg, "through": p_mid_neg, "end": p_psi_neg, "connect": true }))?;
        let l_bot = run_cmd("sketch.line", json!({ "points": [p_psi_neg, bot_start], "connect": true }))?;
        let l_top = run_cmd("sketch.line", json!({ "points": [top_start, p_psi_pos], "connect": true }))?;
        let a_top = run_cmd("sketch.arc.three_point", json!({ "start": p_psi_pos, "through": p_mid_pos, "end": p_hp_pos, "connect": true }))?;
        entities.push(extract_created_curve_id(&a_bot, "bevel bot arc")?);
        entities.push(extract_created_curve_id(&l_bot, "bevel bot line")?);
        entities.push(extract_created_curve_id(&l_top, "bevel top line")?);
        entities.push(extract_created_curve_id(&a_top, "bevel top arc")?);
    } else {
        let psi_f = psi - (t_start - libm::atan(t_start));
        let p_mid_neg = pt_rf(phase_rad - 0.5 * (hp + psi_f));
        let p_mid_pos = pt_rf(phase_rad + 0.5 * (hp + psi_f));
        let a_bot = run_cmd("sketch.arc.three_point", json!({ "start": p_hp_neg, "through": p_mid_neg, "end": bot_start, "connect": true }))?;
        let a_top = run_cmd("sketch.arc.three_point", json!({ "start": top_start, "through": p_mid_pos, "end": p_hp_pos, "connect": true }))?;
        entities.push(extract_created_curve_id(&a_bot, "bevel bot arc")?);
        entities.push(extract_created_curve_id(&a_top, "bevel top arc")?);
    }

    run_cmd(
        "sketch.pattern.circular",
        json!({ "entities": entities, "center": [0.0, 0.0], "count": teeth, "angle": "360 deg" }),
    )?;
    run_cmd("sketch.finish", json!({}))?;
    pick_outer_profile_index(sketch_name, false)
}

fn build_single_bevel(
    existing_names: &mut BTreeSet<String>,
    prefix: &str,
    frame: PlacementFrame,
    re: f64,
    ri: f64,
    delta_rad: f64,
    module: f64,
    teeth: u32,
    alpha_deg: f64,
    half_backlash: f64,
    phase_rad: f64,
    is_spiral: bool,
    spiral_angle_deg: f64,
    hand_sign: f64,
    bore_dia: f64,
    body_name: &str,
) -> Result<String, PluginError> {
    let bname = unique_name(body_name, existing_names);
    let n_sections: usize = if is_spiral { 4 } else { 2 };
    let beta_rad = spiral_angle_deg * core::f64::consts::PI / 180.0;
    let spiral_k = if is_spiral {
        hand_sign * libm::tan(beta_rad) / libm::sin(delta_rad).max(0.15)
    } else {
        0.0
    };

    let mut sections = Vec::new();
    for i in 0..n_sections {
        let u = (i as f64) / ((n_sections - 1) as f64);
        let r_cur = re + u * (ri - re);
        let scale = r_cur / re;
        let z_cur = r_cur * libm::cos(delta_rad);
        let twist = if is_spiral {
            spiral_k * libm::log(re / r_cur)
        } else {
            0.0
        };
        let sk = unique_name(&format!("{prefix}_Sec{i}"), existing_names);
        let prof = draw_bevel_section(
            &sk,
            frame,
            z_cur,
            module * scale,
            teeth,
            alpha_deg,
            half_backlash * scale,
            phase_rad + twist,
        )?;
        sections.push(json!({ "sketch": sk, "profiles": [prof] }));
    }

    let loft_res = run_cmd(
        "solid.loft",
        json!({
            "sections": sections,
            "operation": "new_body",
            "body_name": bname,
        }),
    )?;
    let actual = loft_res
        .get("created_bodies")
        .and_then(Value::as_array)
        .and_then(|a| a.first())
        .and_then(Value::as_str)
        .unwrap_or(&bname)
        .to_string();

    if bore_dia > 0.0 {
        let z_out = re * libm::cos(delta_rad);
        let z_in = ri * libm::cos(delta_rad);
        let z_min = if z_in < z_out { z_in } else { z_out } - 1.0;
        let h = libm::fabs(z_out - z_in) + 2.0;
        let bore_origin = frame.world_point(0.0, 0.0, z_min);
        run_cmd(
            "solid.cylinder",
            json!({
                "diameter": bore_dia,
                "height": h,
                "base": bore_origin,
                "axis": [frame.z_dir[0], frame.z_dir[1], frame.z_dir[2]],
                "operation": "cut",
                "targets": [&actual],
            }),
        )?;
    }
    Ok(actual)
}

pub fn run_bevel_pair(raw: &Value) -> Result<String, PluginError> {
    let mut inp: BevelPairInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    if inp.module <= 0.0 || inp.face_width <= 0.5 {
        return Err(invalid("module", "module and face_width must be positive"));
    }
    if inp.sizing_mode == "by_ratio" {
        if !(0.25..=6.0).contains(&inp.gear_ratio) {
            return Err(invalid("gear_ratio", "bevel gear_ratio must be between 0.25 and 6.0"));
        }
        inp.teeth_2 = (libm::round(inp.teeth_1 as f64 * inp.gear_ratio) as u32).clamp(10, 60);
    }
    if inp.teeth_1 < 10 || inp.teeth_1 > 60 || inp.teeth_2 < 10 || inp.teeth_2 > 60 {
        return Err(invalid("teeth_1", "bevel tooth counts must be between 10 and 60"));
    }
    if !(30.0..=150.0).contains(&inp.shaft_angle) {
        return Err(invalid("shaft_angle", "shaft_angle must be between 30 and 150 deg"));
    }
    let is_spiral = inp.bevel_type == "spiral";
    if is_spiral && !(5.0..=45.0).contains(&inp.spiral_angle) {
        return Err(invalid("spiral_angle", "spiral_angle must be between 5 and 45 deg"));
    }
    let hand1 = if inp.hand == "left" { -1.0 } else { 1.0 };
    let hand2 = -hand1;

    let sigma = inp.shaft_angle * core::f64::consts::PI / 180.0;
    let z1 = inp.teeth_1 as f64;
    let z2 = inp.teeth_2 as f64;
    let delta1 = libm::atan2(libm::sin(sigma), (z2 / z1) + libm::cos(sigma));
    let delta2 = sigma - delta1;
    let re = inp.module * z1 / (2.0 * libm::sin(delta1));
    let max_fw = 0.40 * re;
    let fw = if inp.face_width > max_fw { max_fw } else { inp.face_width };
    let ri = re - fw;

    let frame1 = resolve_placement(
        &inp.placement,
        &inp.axis_name,
        &inp.base_plane,
        inp.flip_direction,
        inp.axial_offset,
        inp.layout_angle,
    )?;
    let frame2 = frame1.rotated_y(sigma);

    let mut existing = list_existing_params()?;
    let mut existing_names = list_existing_doc_names()?;
    let p = choose_unique_prefix(&p, &existing, &existing_names);

    upsert_param(&mut existing, &format!("{p}_m"), &fmt_num(inp.module), "mm", "Bevel outer module", true)?;
    upsert_param(&mut existing, &format!("{p}_z1"), &format!("{}", inp.teeth_1), "", "Pinion teeth", true)?;
    upsert_param(&mut existing, &format!("{p}_z2"), &format!("{}", inp.teeth_2), "", "Gear teeth", true)?;
    upsert_param(&mut existing, &format!("{p}_sigma"), &fmt_num(inp.shaft_angle), "deg", "Shaft angle", true)?;
    upsert_param(&mut existing, &format!("{p}_cone_dist"), &fmt_num(re), "mm", "Outer cone distance Re", true)?;

    let b1 = build_single_bevel(
        &mut existing_names,
        &format!("{p}_p1"),
        frame1,
        re,
        ri,
        delta1,
        inp.module,
        inp.teeth_1,
        inp.pressure_angle,
        inp.backlash / 2.0,
        0.0,
        is_spiral,
        inp.spiral_angle,
        hand1,
        inp.bore_1,
        &format!("{p}_BevelPinion"),
    )?;

    let phase2 = core::f64::consts::PI + core::f64::consts::PI / z2;
    let b2 = build_single_bevel(
        &mut existing_names,
        &format!("{p}_p2"),
        frame2,
        re,
        ri,
        delta2,
        inp.module,
        inp.teeth_2,
        inp.pressure_angle,
        inp.backlash / 2.0,
        phase2,
        is_spiral,
        inp.spiral_angle,
        hand2,
        inp.bore_2,
        &format!("{p}_BevelGear"),
    )?;

    Ok(json!({
        "mode": "gm-bevel-pair",
        "bevel_type": inp.bevel_type,
        "prefix": p,
        "ratio": z2 / z1,
        "cone_distance_mm": re,
        "pitch_cone_1_deg": delta1 * 180.0 / core::f64::consts::PI,
        "pitch_cone_2_deg": delta2 * 180.0 / core::f64::consts::PI,
        "pinion_body": b1,
        "gear_body": b2,
    })
    .to_string())
}

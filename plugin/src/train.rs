//! Multi-Stage Compound Gear Train generator (`gm-gear-train`).
//! Automatically splits a target total gear ratio across 2–4 balanced stages,
//! lays out coaxial compound intermediate shafts (Linear or Folded), computes
//! stage-by-stage torque & Lewis/Hertz stress checks, and optionally emits
//! assembly components + `joint.gear` / rigid compound-shaft joints.

use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use serde::Deserialize;
use serde_json::{Value, json};

use crate::cylindrical::{GearSide, GearSpec, ToothKind, build_parametric_gear, setup_transverse_params};
use crate::placement::{PlacementFrame, resolve_placement};
use crate::{
    PluginError, choose_unique_prefix, fmt_num, invalid, list_existing_doc_names,
    list_existing_params, unique_name, upsert_param, validate_prefix,
};

#[derive(Deserialize)]
#[serde(default)]
struct GearTrainInput {
    placement: Vec<Value>,
    base_plane: String,
    layout_angle: f64,
    flip_direction: bool,
    axial_offset: f64,
    tooth_type: String,
    helix_angle: f64,
    hand: String,
    module_system: String,
    prefix: String,
    num_stages: u32,
    total_ratio: f64,
    layout_style: String,
    sizing_unit: String,
    module: f64,
    diametral_pitch: f64,
    circular_pitch: f64,
    min_pinion_teeth: u32,
    max_gear_teeth: u32,
    pressure_angle: f64,
    face_width: f64,
    stage_gap: f64,
    bore_diameter: f64,
    backlash: f64,
    root_fillet: bool,
    create_joints: bool,
    material: String,
    input_torque_nm: f64,
    input_rpm: f64,
}

impl Default for GearTrainInput {
    fn default() -> Self {
        Self {
            placement: Vec::new(),
            base_plane: "XY".into(),
            layout_angle: 0.0,
            flip_direction: false,
            axial_offset: 0.0,
            tooth_type: "straight".into(),
            helix_angle: 15.0,
            hand: "right".into(),
            module_system: "normal".into(),
            prefix: "tr1".into(),
            num_stages: 2,
            total_ratio: 12.0,
            layout_style: "folded".into(),
            sizing_unit: "module".into(),
            module: 1.5,
            diametral_pitch: 16.0,
            circular_pitch: 4.712,
            min_pinion_teeth: 14,
            max_gear_teeth: 72,
            pressure_angle: 20.0,
            face_width: 10.0,
            stage_gap: 3.0,
            bore_diameter: 6.0,
            backlash: 0.05,
            root_fillet: false,
            create_joints: false,
            material: "steel_1045".into(),
            input_torque_nm: 0.0,
            input_rpm: 0.0,
        }
    }
}

/// Split `total_ratio` across `num_stages` balanced `(zp, zg)` tooth pairs.
/// When `coaxial_folded` is true, ALL stages are constrained to the exact same tooth sum
/// `zp + zg = Z_sum` so every shaft in a folded 2-shaft gearbox is 100% coaxial!
pub fn split_train_stages(
    num_stages: u32,
    total_ratio: f64,
    min_pinion: u32,
    max_gear: u32,
    coaxial_folded: bool,
) -> Vec<(u32, u32)> {
    let n = num_stages.clamp(2, 4);
    if coaxial_folded {
        let ideal_u = libm::pow(total_ratio.max(1.5), 1.0 / (n as f64));
        let base_zp = min_pinion;
        let base_zg = (libm::round(base_zp as f64 * ideal_u) as u32).clamp(base_zp, max_gear);
        let center_sum = base_zp + base_zg;
        let lo_sum = center_sum.saturating_sub(6).max(2 * min_pinion);
        let hi_sum = (center_sum + 8).min(min_pinion + max_gear);

        let mut best_stages: Vec<(u32, u32)> = Vec::new();
        let mut best_total_err = f64::MAX;

        for z_sum in lo_sum..=hi_sum {
            let mut cand = Vec::with_capacity(n as usize);
            let mut rem = total_ratio.max(1.5);
            let mut valid = true;
            for i in 0..n {
                let left = (n - i) as f64;
                let target_u = libm::pow(rem, 1.0 / left);
                // Solve zp + zg = z_sum, zg / zp ≈ target_u => zp ≈ z_sum / (1 + target_u)
                let ideal_zp = libm::round((z_sum as f64) / (1.0 + target_u)) as u32;
                let max_zp = (z_sum / 2).min(max_gear);
                let min_zp = min_pinion.max(z_sum.saturating_sub(max_gear));
                if min_zp > max_zp {
                    valid = false;
                    break;
                }
                let zp = ideal_zp.clamp(min_zp, max_zp);
                let zg = z_sum - zp;
                let u = zg as f64 / zp as f64;
                rem = (rem / u).max(1.0);
                cand.push((zp, zg));
            }
            if !valid {
                continue;
            }
            let prod: f64 = cand.iter().map(|&(p, g)| g as f64 / p as f64).product();
            let err = libm::fabs(prod - total_ratio) / total_ratio;
            if err < best_total_err {
                best_total_err = err;
                best_stages = cand;
            }
        }
        if !best_stages.is_empty() {
            return best_stages;
        }
    }

    let mut stages = Vec::with_capacity(n as usize);
    let mut rem_ratio = total_ratio.max(1.5);

    for i in 0..n {
        let stages_left = (n - i) as f64;
        let target_u = libm::pow(rem_ratio, 1.0 / stages_left);

        let mut best_pair = (
            min_pinion,
            (libm::round(min_pinion as f64 * target_u) as u32).clamp(min_pinion, max_gear),
        );
        let mut best_err = f64::MAX;

        let p_hi = (min_pinion + 4).min(max_gear);
        for zp in min_pinion..=p_hi {
            let zg = (libm::round(zp as f64 * target_u) as u32).clamp(zp, max_gear);
            let actual_u = zg as f64 / zp as f64;
            let err = libm::fabs(actual_u - target_u) / target_u;
            if err < best_err {
                best_err = err;
                best_pair = (zp, zg);
            }
        }
        let stage_u = best_pair.1 as f64 / best_pair.0 as f64;
        rem_ratio = (rem_ratio / stage_u).max(1.0);
        stages.push(best_pair);
    }
    stages
}

fn shift_frame_z(frame: PlacementFrame, dz: f64) -> PlacementFrame {
    PlacementFrame {
        origin: [
            frame.origin[0] + dz * frame.z_dir[0],
            frame.origin[1] + dz * frame.z_dir[1],
            frame.origin[2] + dz * frame.z_dir[2],
        ],
        x_dir: frame.x_dir,
        y_dir: frame.y_dir,
        z_dir: frame.z_dir,
    }
}

pub fn run_gear_train(raw: &Value) -> Result<String, PluginError> {
    let mut inp: GearTrainInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    inp.module = crate::solver::resolve_module_mm(
        &inp.sizing_unit,
        inp.module,
        inp.diametral_pitch,
        inp.circular_pitch,
    )
    .map_err(|e| invalid("module", e))?;
    if !(2..=4).contains(&inp.num_stages) {
        return Err(invalid("num_stages", "num_stages must be between 2 and 4"));
    }
    if inp.total_ratio < 1.5 || inp.total_ratio > 500.0 {
        return Err(invalid("total_ratio", "total_ratio must be between 1.5 and 500"));
    }
    if inp.min_pinion_teeth < 8 || inp.max_gear_teeth > 120 || inp.min_pinion_teeth > inp.max_gear_teeth {
        return Err(invalid("min_pinion_teeth", "tooth bounds must be 8 <= min <= max <= 120"));
    }
    if inp.face_width <= 0.0 {
        return Err(invalid("face_width", "face_width must be positive"));
    }

    crate::report_progress(0.05, "Solving multi-stage gear train ratios")?;
    let folded = inp.layout_style == "folded";
    let stages = split_train_stages(
        inp.num_stages,
        inp.total_ratio,
        inp.min_pinion_teeth,
        inp.max_gear_teeth,
        folded,
    );
    let tooth_kind = ToothKind::parse(&inp.tooth_type);
    let hand_sign_base = if inp.hand == "left" { -1.0 } else { 1.0 };
    let base_frame = resolve_placement(
        &inp.placement,
        "",
        &inp.base_plane,
        inp.flip_direction,
        inp.axial_offset,
        inp.layout_angle,
    )?;

    let mut existing = list_existing_params()?;
    let mut existing_names = list_existing_doc_names()?;
    let p = choose_unique_prefix(&p, &existing, &existing_names);

    let m_p = format!("{p}_m");
    let a_p = format!("{p}_alpha");
    let fw_p = format!("{p}_fw");
    let bl_p = format!("{p}_bl");
    let bore_p = format!("{p}_bore");
    let x_zero_p = format!("{p}_x0");

    upsert_param(&mut existing, &m_p, &fmt_num(inp.module), "mm", "Train module", true)?;
    upsert_param(&mut existing, &a_p, &fmt_num(inp.pressure_angle), "deg", "Pressure angle", true)?;
    upsert_param(&mut existing, &fw_p, &fmt_num(inp.face_width), "mm", "Stage face width", true)?;
    upsert_param(&mut existing, &bl_p, &fmt_num(inp.backlash * 0.5), "mm", "Half backlash per gear", false)?;
    upsert_param(&mut existing, &x_zero_p, "0", "", "Zero profile shift", false)?;
    if inp.bore_diameter > 0.0 {
        upsert_param(&mut existing, &bore_p, &fmt_num(inp.bore_diameter), "mm", "Shaft bore", true)?;
    }

    let (mt_p, at_p, beta_p, mt_val, at_deg) = setup_transverse_params(
        &mut existing,
        &p,
        &m_p,
        &a_p,
        tooth_kind,
        inp.helix_angle,
        &inp.module_system,
        inp.module,
        inp.pressure_angle,
    )?;

    let preview_only = crate::is_preview(raw);
    let mut shaft_x_mm = 0.0_f64;
    let mut dir_sign = 1.0_f64;
    let mut actual_total_ratio = 1.0_f64;
    let mut stage_reports: Vec<Value> = Vec::new();
    let mut all_bodies: Vec<String> = Vec::new();
    // Track per-stage (pinion_body, gear_body, zp_param, zg_param, shaft_a_pos, shaft_b_pos)
    let mut joint_specs: Vec<(String, String, String, String, [f64; 3], [f64; 3])> = Vec::new();

    for (idx, &(zp, zg)) in stages.iter().enumerate() {
        let s_num = idx + 1;
        let frac = 0.1 + 0.8 * (idx as f64 / stages.len() as f64);
        crate::report_progress(frac as f32, &format!("Building Stage {s_num} ({zp}T -> {zg}T)"))?;

        let dz = (idx as f64) * (inp.face_width + inp.stage_gap);
        let stage_frame = shift_frame_z(base_frame, dz);
        let cd_mm = mt_val * (zp + zg) as f64 / 2.0;
        let next_x_mm = shaft_x_mm + dir_sign * cd_mm;

        let zp_p = format!("{p}_s{s_num}_zp");
        let zg_p = format!("{p}_s{s_num}_zg");
        let xp_p = format!("{p}_s{s_num}_xp");
        let xg_p = format!("{p}_s{s_num}_xg");
        let phi_p_p = format!("{p}_s{s_num}_phip");
        let phi_g_p = format!("{p}_s{s_num}_phig");

        upsert_param(&mut existing, &zp_p, &format!("{zp}"), "", &format!("Stage {s_num} pinion teeth"), true)?;
        upsert_param(&mut existing, &zg_p, &format!("{zg}"), "", &format!("Stage {s_num} gear teeth"), true)?;
        upsert_param(&mut existing, &xp_p, &fmt_num(shaft_x_mm), "mm", &format!("Stage {s_num} pinion X"), false)?;
        upsert_param(&mut existing, &xg_p, &fmt_num(next_x_mm), "mm", &format!("Stage {s_num} gear X"), false)?;

        // Always orient Pinion tooth #0 toward Gear (+X when dir_sign > 0, -X when dir_sign < 0),
        // and orient Gear with a tooth space facing Pinion. Works identically for even & odd tooth counts.
        let (pinion_phi_opt, gear_phase_expr) = if dir_sign > 0.0 {
            (None, format!("pi + pi / {zg_p}"))
        } else {
            upsert_param(&mut existing, &phi_p_p, "pi", "", &format!("Stage {s_num} pinion phase (rad)"), false)?;
            (Some(phi_p_p.as_str()), format!("pi / {zg_p}"))
        };
        upsert_param(&mut existing, &phi_g_p, &gear_phase_expr, "", &format!("Stage {s_num} mesh phase (rad)"), false)?;

        let p_body = unique_name(&format!("{p}_S{s_num}_Pinion"), &mut existing_names);
        let p_sk = unique_name(&format!("{p}_S{s_num}_PinionSk"), &mut existing_names);
        let p_path = unique_name(&format!("{p}_S{s_num}_PinionAxis"), &mut existing_names);
        let g_body = unique_name(&format!("{p}_S{s_num}_Gear"), &mut existing_names);
        let g_sk = unique_name(&format!("{p}_S{s_num}_GearSk"), &mut existing_names);
        let g_path = unique_name(&format!("{p}_S{s_num}_GearAxis"), &mut existing_names);

        let p_info = build_parametric_gear(
            &mut existing,
            &GearSpec {
                derived_prefix: &format!("{p}_s{s_num}p"),
                m_param: &m_p,
                mt_param: &mt_p,
                at_param: &at_p,
                z_param: &zp_p,
                fw_param: &fw_p,
                x_param: &x_zero_p,
                bl_param: &bl_p,
                bore_param: if inp.bore_diameter > 0.0 { Some(&bore_p) } else { None },
                rim_param: None,
                phi_param: pinion_phi_opt,
                gear_side: GearSide::External,
                tooth_kind,
                beta_param: beta_p.as_deref(),
                hand_sign: hand_sign_base,
                frame: stage_frame,
                offset_x_param: Some(&xp_p),
                sketch_name: &p_sk,
                path_sketch_name: &p_path,
                body_name: &p_body,
                m_val: inp.module,
                mt_val,
                at_deg,
                teeth_val: zp,
                shift_val: 0.0,
                bore_val: inp.bore_diameter,
                rim_val: 0.0,
                root_fillet: inp.root_fillet,
                preview_only,
            },
        )?;

        let g_info = build_parametric_gear(
            &mut existing,
            &GearSpec {
                derived_prefix: &format!("{p}_s{s_num}g"),
                m_param: &m_p,
                mt_param: &mt_p,
                at_param: &at_p,
                z_param: &zg_p,
                fw_param: &fw_p,
                x_param: &x_zero_p,
                bl_param: &bl_p,
                bore_param: if inp.bore_diameter > 0.0 { Some(&bore_p) } else { None },
                rim_param: None,
                phi_param: Some(&phi_g_p),
                gear_side: GearSide::External,
                tooth_kind,
                beta_param: beta_p.as_deref(),
                hand_sign: -hand_sign_base,
                frame: stage_frame,
                offset_x_param: Some(&xg_p),
                sketch_name: &g_sk,
                path_sketch_name: &g_path,
                body_name: &g_body,
                m_val: inp.module,
                mt_val,
                at_deg,
                teeth_val: zg,
                shift_val: 0.0,
                bore_val: inp.bore_diameter,
                rim_val: 0.0,
                root_fillet: inp.root_fillet,
                preview_only,
            },
        )?;

        let actual_p_body = p_info.get("body").and_then(Value::as_str).unwrap_or(&p_body).to_string();
        let actual_g_body = g_info.get("body").and_then(Value::as_str).unwrap_or(&g_body).to_string();
        all_bodies.push(actual_p_body.clone());
        all_bodies.push(actual_g_body.clone());

        let pos_p = crate::joints::offset_point(&stage_frame, shaft_x_mm, 0.0, 0.0);
        let pos_g = crate::joints::offset_point(&stage_frame, next_x_mm, 0.0, 0.0);
        joint_specs.push((actual_p_body, actual_g_body, zp_p, zg_p, pos_p, pos_g));

        // Compute stage torque & preliminary stress check
        let stage_torque = inp.input_torque_nm * actual_total_ratio;
        let stage_rpm = if actual_total_ratio > 0.0 { inp.input_rpm / actual_total_ratio } else { 0.0 };
        let beta_deg = if tooth_kind == ToothKind::Straight { 0.0 } else { inp.helix_angle };
        let sol = crate::solver::solve_external_pair(
            inp.module,
            mt_val,
            zp,
            zg,
            inp.pressure_angle,
            at_deg,
            beta_deg,
            0.0,
            0.0,
            inp.face_width,
            0.0,
            &inp.material,
            stage_torque,
            stage_rpm,
        );

        let stage_u = zg as f64 / zp as f64;
        actual_total_ratio *= stage_u;
        stage_reports.push(json!({
            "stage": s_num,
            "pinion_teeth": zp,
            "gear_teeth": zg,
            "stage_ratio": stage_u,
            "center_distance_mm": cd_mm,
            "stage_torque_nm": stage_torque,
            "engineering": sol.to_json(),
        }));

        shaft_x_mm = next_x_mm;
        if folded {
            dir_sign = -dir_sign;
        }
    }

    let mut joints_created = false;
    if inp.create_joints && !preview_only {
        crate::report_progress(0.92, "Creating assembly components & gear joints")?;
        crate::joints::make_components_from_bodies(&all_bodies)?;
        for (idx, (p_b, g_b, zp_p, zg_p, pos_p, pos_g)) in joint_specs.iter().enumerate() {
            let s_num = idx + 1;
            let jp = format!("{p}_S{s_num}_JP");
            let jg = format!("{p}_S{s_num}_JG");
            if idx == 0 {
                crate::joints::add_revolute_joint(&jp, p_b, *pos_p, &base_frame)?;
            } else {
                // Compound intermediate shaft: lock this stage's pinion to previous stage's gear
                let prev_g = &joint_specs[idx - 1].1;
                crate::joints::add_rigid_pair(p_b, prev_g, &format!("{p}_S{s_num}_ShaftLock"))?;
            }
            crate::joints::add_revolute_joint(&jg, g_b, *pos_g, &base_frame)?;
            let driver_joint = if idx == 0 { jp } else { format!("{p}_S{idx}_JG") };
            crate::joints::add_gear_link(&driver_joint, &jg, zp_p, zg_p, false)?;
        }
        joints_created = true;
    }

    crate::report_progress(1.0, "Multi-stage gear train complete")?;
    Ok(json!({
        "mode": "gm-gear-train",
        "prefix": p,
        "num_stages": inp.num_stages,
        "target_ratio": inp.total_ratio,
        "actual_total_ratio": actual_total_ratio,
        "layout_style": inp.layout_style,
        "resolved_module_mm": inp.module,
        "joints_created": joints_created,
        "stages": stage_reports,
    })
    .to_string())
}

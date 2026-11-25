//! Smart `dialog@0.1` engineering hook for GM Gear Suite.
//! Computes material-aware defaults (backlash, root fillet, anti-undercut profile shift,
//! and load-based ISO 54 module / face-width recommendations) plus live engineering hints.
//! Note: SolidCraft's `dialog@0.1` host never overwrites a field the user manually edited.

use alloc::format;
use serde_json::{Map, Value, json};

use crate::solver;

/// Recommended backlash (mm) and root-fillet default for each material preset.
fn material_smart_defaults(material_id: &str) -> (f64, bool, &'static str) {
    match material_id {
        "petg_3d" => (0.20, false, "FDM 3D-print clearance (0.20 mm)"),
        "pom_delrin" => (0.08, false, "POM / Delrin clearance (0.08 mm)"),
        "pa66_nylon" => (0.08, false, "PA66 Nylon clearance (0.08 mm)"),
        "peek" => (0.06, false, "PEEK precision clearance (0.06 mm)"),
        "brass_c360" => (0.06, false, "Brass C360 clearance (0.06 mm)"),
        "steel_4140" => (0.05, true, "Steel 4140 QT (0.05 mm + 0.3m root fillet)"),
        _ => (0.05, true, "Steel 1045 (0.05 mm + 0.3m root fillet)"),
    }
}

/// Find the smallest ISO 54 Series 1 module (with b = 10*m) that achieves SF >= 1.5 and SH >= 1.2.
#[allow(clippy::too_many_arguments)]
fn recommend_module_for_load(
    z1: u32,
    z2: u32,
    alpha_n_deg: f64,
    beta_deg: f64,
    is_normal: bool,
    x1: f64,
    x2: f64,
    material_id: &str,
    torque_nm: f64,
    rpm: f64,
) -> (f64, f64) {
    let cos_b = libm::cos(beta_deg.to_radians());
    for &m_cand in solver::ISO54_SERIES1 {
        let (mt, at_deg) = if beta_deg <= 0.1 || !is_normal {
            (m_cand, alpha_n_deg)
        } else {
            (
                m_cand / cos_b,
                libm::atan(libm::tan(alpha_n_deg.to_radians()) / cos_b).to_degrees(),
            )
        };
        let fw = (10.0 * m_cand).clamp(2.0, 500.0);
        let sol = solver::solve_external_pair(
            m_cand,
            mt,
            z1,
            z2,
            alpha_n_deg,
            at_deg,
            beta_deg,
            x1,
            x2,
            fw,
            0.0,
            material_id,
            torque_nm,
            rpm,
        );
        if let Some(s) = sol.strength {
            let sfb = s
                .get("bending_safety_factor")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            let sfh = s
                .get("contact_safety_factor")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            if sfb >= 1.5 && sfh >= 1.2 {
                return (m_cand, fw);
            }
        }
    }
    (20.0, 200.0)
}

fn round2(v: f64) -> f64 {
    libm::round(v * 100.0) / 100.0
}

/// Live `dialog@0.1` hook: computes material-smart defaults and live engineering hints.
pub fn compute_dialog_hints(entry: &str, values: &Value) -> Value {
    let mut patches = Map::new();

    // Shared material-driven backlash default for any gear tool that exposes `material` + `backlash`.
    let mat = values
        .get("material")
        .and_then(Value::as_str)
        .unwrap_or("steel_1045");
    let (smart_bl, smart_fillet, mat_note) = material_smart_defaults(mat);

    match entry {
        "gm-cylindrical-gear" => {
            patches.insert(
                "backlash".into(),
                json!({ "default": smart_bl, "hint": mat_note }),
            );
            patches.insert(
                "root_fillet".into(),
                json!({ "default": smart_fillet }),
            );

            let sizing_unit = values
                .get("sizing_unit")
                .and_then(Value::as_str)
                .unwrap_or("module");
            let m_in = values.get("module").and_then(Value::as_f64).unwrap_or(2.0);
            let dp = values
                .get("diametral_pitch")
                .and_then(Value::as_f64)
                .unwrap_or(12.7);
            let cp = values
                .get("circular_pitch")
                .and_then(Value::as_f64)
                .unwrap_or(6.283);
            let Ok(m_n) = solver::resolve_module_mm(sizing_unit, m_in, dp, cp) else {
                return Value::Object(patches);
            };
            let z = values.get("teeth").and_then(Value::as_u64).unwrap_or(20) as u32;
            let alpha = values
                .get("pressure_angle")
                .and_then(Value::as_f64)
                .unwrap_or(20.0);
            let fw = values
                .get("face_width")
                .and_then(Value::as_f64)
                .unwrap_or(10.0);
            let x = values
                .get("profile_shift")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            let tooth_type = values
                .get("tooth_type")
                .and_then(Value::as_str)
                .unwrap_or("straight");
            let beta = if tooth_type == "straight" {
                0.0
            } else {
                values
                    .get("helix_angle")
                    .and_then(Value::as_f64)
                    .unwrap_or(15.0)
            };
            let mod_sys = values
                .get("module_system")
                .and_then(Value::as_str)
                .unwrap_or("normal");
            let gear_side = values
                .get("gear_side")
                .and_then(Value::as_str)
                .unwrap_or("external");
            let torque = values
                .get("input_torque_nm")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            let rpm = values
                .get("input_rpm")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);

            let cos_b = libm::cos(beta.to_radians());
            let (m_t, at_deg) = if tooth_type == "straight" || mod_sys == "transverse" {
                (m_n, alpha)
            } else {
                (
                    m_n / cos_b,
                    libm::atan(libm::tan(alpha.to_radians()) / cos_b).to_degrees(),
                )
            };
            let d = m_t * (z as f64);
            let da = if gear_side == "internal" {
                d - 2.0 * m_n * (1.0 - x)
            } else {
                d + 2.0 * m_n * (1.0 + x)
            };
            let df = if gear_side == "internal" {
                d + 2.0 * m_n * (1.25 + x)
            } else {
                d - 2.0 * m_n * (1.25 - x)
            };

            if torque > 0.0 && sizing_unit == "module" {
                let (rec_m, rec_fw) = recommend_module_for_load(
                    z,
                    z,
                    alpha,
                    beta,
                    mod_sys == "normal",
                    x,
                    x,
                    mat,
                    torque,
                    rpm,
                );
                patches.insert(
                    "module".into(),
                    json!({
                        "default": rec_m,
                        "hint": format!("Smart ISO 54 sizing for {torque:.1} N·m: m >= {rec_m} mm, b >= {rec_fw:.0} mm")
                    }),
                );
                patches.insert(
                    "face_width".into(),
                    json!({ "default": rec_fw }),
                );
            } else if sizing_unit == "dp" {
                patches.insert(
                    "diametral_pitch".into(),
                    json!({ "hint": format!("Equivalent module m = {m_n:.3} mm") }),
                );
            } else if sizing_unit == "cp" {
                patches.insert(
                    "circular_pitch".into(),
                    json!({ "hint": format!("Equivalent module m = {m_n:.3} mm") }),
                );
            } else {
                let nearest = solver::nearest_iso54_module(m_n);
                if libm::fabs(m_n - nearest) > 1e-3 {
                    patches.insert(
                        "module".into(),
                        json!({ "hint": format!("Non-ISO 54 Series 1 (nearest standard: {nearest} mm)") }),
                    );
                }
            }

            patches.insert(
                "teeth".into(),
                json!({ "hint": format!("Pitch d = {d:.2} mm | Tip da = {da:.2} mm | Root df = {df:.2} mm") }),
            );

            if gear_side == "external" {
                let sin_a = libm::sin(alpha.to_radians());
                let z_min = (2.0 / (sin_a * sin_a)) * cos_b * cos_b * cos_b;
                let x_min = ((z_min - z as f64) / z_min).max(0.0);
                if (z as f64) < z_min {
                    let rec_x = round2(x_min).clamp(0.0, 1.0);
                    patches.insert(
                        "profile_shift".into(),
                        json!({
                            "default": rec_x,
                            "hint": format!("Undercut guard (z < {z_min:.1}): auto-suggested x = +{rec_x:.2}")
                        }),
                    );
                }
            }

            if torque > 0.0 {
                let r = solver::solve_external_pair(
                    m_n, m_t, z, z, alpha, at_deg, beta, x, x, fw, 0.0, mat, torque, rpm,
                );
                if let Some(s) = &r.strength {
                    let sfb = s.get("bending_safety_factor").and_then(Value::as_f64).unwrap_or(0.0);
                    let sb = s.get("lewis_bending_mpa").and_then(Value::as_f64).unwrap_or(0.0);
                    let sfh = s.get("contact_safety_factor").and_then(Value::as_f64).unwrap_or(0.0);
                    let sh = s.get("hertz_contact_mpa").and_then(Value::as_f64).unwrap_or(0.0);
                    let verdict = if sfb >= 1.5 && sfh >= 1.2 {
                        "PASS"
                    } else if sfb >= 1.0 && sfh >= 1.0 {
                        "MARGINAL"
                    } else {
                        "UNDER-RATED"
                    };
                    patches.insert(
                        "input_torque_nm".into(),
                        json!({ "hint": format!("{verdict}: Bending SF={sfb:.2} ({sb:.0} MPa), Contact SH={sfh:.2} ({sh:.0} MPa)") }),
                    );
                }
            }
        }
        "gm-parallel-pair" => {
            patches.insert(
                "backlash".into(),
                json!({ "default": smart_bl, "hint": mat_note }),
            );
            patches.insert(
                "root_fillet".into(),
                json!({ "default": smart_fillet }),
            );

            let sizing_unit = values
                .get("sizing_unit")
                .and_then(Value::as_str)
                .unwrap_or("module");
            let m_in = values.get("module").and_then(Value::as_f64).unwrap_or(2.0);
            let dp = values
                .get("diametral_pitch")
                .and_then(Value::as_f64)
                .unwrap_or(12.7);
            let cp = values
                .get("circular_pitch")
                .and_then(Value::as_f64)
                .unwrap_or(6.283);
            let Ok(m_n) = solver::resolve_module_mm(sizing_unit, m_in, dp, cp) else {
                return Value::Object(patches);
            };
            let z1 = values.get("teeth_1").and_then(Value::as_u64).unwrap_or(16) as u32;
            let sizing_mode = values
                .get("sizing_mode")
                .and_then(Value::as_str)
                .unwrap_or("by_teeth");
            let z2 = if sizing_mode == "by_ratio" {
                let r = values
                    .get("gear_ratio")
                    .and_then(Value::as_f64)
                    .unwrap_or(2.0);
                libm::round((z1 as f64) * r).max(8.0) as u32
            } else {
                values.get("teeth_2").and_then(Value::as_u64).unwrap_or(32) as u32
            };
            let alpha = values
                .get("pressure_angle")
                .and_then(Value::as_f64)
                .unwrap_or(20.0);
            let fw = values
                .get("face_width")
                .and_then(Value::as_f64)
                .unwrap_or(10.0);
            let x1 = values
                .get("profile_shift_1")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            let x2 = values
                .get("profile_shift_2")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            let tooth_type = values
                .get("tooth_type")
                .and_then(Value::as_str)
                .unwrap_or("straight");
            let beta = if tooth_type == "straight" {
                0.0
            } else {
                values
                    .get("helix_angle")
                    .and_then(Value::as_f64)
                    .unwrap_or(15.0)
            };
            let mod_sys = values
                .get("module_system")
                .and_then(Value::as_str)
                .unwrap_or("normal");
            let torque = values
                .get("input_torque_nm")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            let rpm = values
                .get("input_rpm")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);

            let cos_b = libm::cos(beta.to_radians());
            let (m_t, alpha_t_deg) = if tooth_type == "straight" || mod_sys == "transverse" {
                (m_n, alpha)
            } else {
                let at = libm::atan(libm::tan(alpha.to_radians()) / cos_b).to_degrees();
                (m_n / cos_b, at)
            };
            let r = solver::solve_external_pair(
                m_n,
                m_t,
                z1,
                z2,
                alpha,
                alpha_t_deg,
                beta,
                x1,
                x2,
                fw,
                0.0,
                mat,
                torque,
                rpm,
            );

            if torque > 0.0 && sizing_unit == "module" {
                let (rec_m, rec_fw) = recommend_module_for_load(
                    z1,
                    z2,
                    alpha,
                    beta,
                    mod_sys == "normal",
                    x1,
                    x2,
                    mat,
                    torque,
                    rpm,
                );
                patches.insert(
                    "module".into(),
                    json!({
                        "default": rec_m,
                        "hint": format!("Smart ISO 54 sizing for {torque:.1} N·m: m >= {rec_m} mm, b >= {rec_fw:.0} mm")
                    }),
                );
                patches.insert("face_width".into(), json!({ "default": rec_fw }));
            } else if sizing_unit == "dp" {
                patches.insert(
                    "diametral_pitch".into(),
                    json!({ "hint": format!("Equivalent module m = {m_n:.3} mm") }),
                );
            } else if sizing_unit == "cp" {
                patches.insert(
                    "circular_pitch".into(),
                    json!({ "hint": format!("Equivalent module m = {m_n:.3} mm") }),
                );
            }

            let ratio = z2 as f64 / z1 as f64;
            let cd_hint = format!(
                "CD aw = {:.3} mm (alpha_w = {:.2} deg) | eps_a = {:.2} (tot {:.2}) | u = {:.3}:1",
                r.operating_center_distance_mm,
                r.operating_pressure_angle_deg,
                r.contact_ratio_transverse,
                r.contact_ratio_total,
                ratio
            );
            if sizing_mode == "by_ratio" {
                patches.insert("gear_ratio".into(), json!({ "hint": cd_hint }));
            } else {
                patches.insert("teeth_2".into(), json!({ "hint": cd_hint }));
            }

            if r.x1_min_recommended > 0.01 {
                let rec_x1 = round2(r.x1_min_recommended).clamp(0.0, 1.0);
                patches.insert(
                    "profile_shift_1".into(),
                    json!({
                        "default": rec_x1,
                        "hint": format!("Pinion undercut guard (z1 < {:.1}): auto-suggested x1 = +{rec_x1:.2}", r.z1_min_no_undercut)
                    }),
                );
            }

            if let Some(s) = &r.strength {
                let sfb = s.get("bending_safety_factor").and_then(Value::as_f64).unwrap_or(0.0);
                let sb = s.get("lewis_bending_mpa").and_then(Value::as_f64).unwrap_or(0.0);
                let sfh = s.get("contact_safety_factor").and_then(Value::as_f64).unwrap_or(0.0);
                let sh = s.get("hertz_contact_mpa").and_then(Value::as_f64).unwrap_or(0.0);
                let verdict = if sfb >= 1.5 && sfh >= 1.2 {
                    "PASS"
                } else if sfb >= 1.0 && sfh >= 1.0 {
                    "MARGINAL"
                } else {
                    "UNDER-RATED"
                };
                patches.insert(
                    "input_torque_nm".into(),
                    json!({ "hint": format!("{verdict}: Bending SF={sfb:.2} ({sb:.0} MPa), Contact SH={sfh:.2} ({sh:.0} MPa)") }),
                );
            }
        }
        "gm-gear-train" => {
            patches.insert(
                "backlash".into(),
                json!({ "default": smart_bl, "hint": mat_note }),
            );
            patches.insert(
                "root_fillet".into(),
                json!({ "default": smart_fillet }),
            );
            let n_stages = values.get("num_stages").and_then(Value::as_u64).unwrap_or(2) as u32;
            let ratio = values.get("total_ratio").and_then(Value::as_f64).unwrap_or(12.0);
            let min_p = values.get("min_pinion_teeth").and_then(Value::as_u64).unwrap_or(14) as u32;
            let max_g = values.get("max_gear_teeth").and_then(Value::as_u64).unwrap_or(72) as u32;
            let stages = crate::train::split_train_stages(n_stages, ratio, min_p, max_g);
            let mut summary = alloc::string::String::new();
            let mut actual = 1.0_f64;
            for (i, (zp, zg)) in stages.iter().enumerate() {
                if i > 0 {
                    summary.push_str(" x ");
                }
                let u = *zg as f64 / *zp as f64;
                actual *= u;
                summary.push_str(&format!("S{}:{}T->{}T({:.2})", i + 1, zp, zg, u));
            }
            patches.insert(
                "total_ratio".into(),
                json!({ "hint": format!("{summary} = {actual:.2}:1") }),
            );
        }
        "gm-planetary" | "gm-rack-pinion" | "gm-bevel-pair" | "gm-worm-drive" => {
            patches.insert(
                "backlash".into(),
                json!({ "default": smart_bl, "hint": mat_note }),
            );
        }
        _ => {}
    }
    Value::Object(patches)
}

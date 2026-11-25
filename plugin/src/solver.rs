//! Pure mechanical engineering solver & validator for GM Gear Suite (no CAD calls).
//! Implements ISO 54 standard modules, ISO 21771 profile-shifted center distance,
//! undercut detection, contact ratio (transverse & helical overlap), face-width checks,
//! and preliminary Lewis bending / Hertzian contact stress estimates.

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use serde_json::{Value, json};

pub const ISO54_SERIES1: &[f64] = &[
    0.5, 0.8, 1.0, 1.25, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0,
];

/// Convert Module (mm), Diametral Pitch (1/in), or Circular Pitch (mm) to metric module (mm).
pub fn resolve_module_mm(sizing_unit: &str, module: f64, dp: f64, cp: f64) -> Result<f64, String> {
    let m = match sizing_unit {
        "dp" => {
            if dp <= 0.0 {
                return Err("diametral_pitch must be positive".into());
            }
            25.4 / dp
        }
        "cp" => {
            if cp <= 0.0 {
                return Err("circular_pitch must be positive".into());
            }
            cp / core::f64::consts::PI
        }
        _ => module,
    };
    if m < 0.1 || m > 60.0 {
        return Err(format!("resolved module ({m:.3} mm) must be between 0.1 and 60 mm"));
    }
    Ok(m)
}

pub fn nearest_iso54_module(m: f64) -> f64 {
    let mut best = ISO54_SERIES1[0];
    let mut best_d = libm::fabs(m - best);
    for &cand in ISO54_SERIES1 {
        let d = libm::fabs(m - cand);
        if d < best_d {
            best_d = d;
            best = cand;
        }
    }
    best
}

pub fn next_iso54_module(min_m: f64) -> f64 {
    for &cand in ISO54_SERIES1 {
        if cand >= min_m {
            return cand;
        }
    }
    *ISO54_SERIES1.last().unwrap_or(&20.0)
}

#[inline]
pub fn inv(angle_rad: f64) -> f64 {
    libm::tan(angle_rad) - angle_rad
}

/// Invert involute function: find alpha in (0, pi/2) such that tan(alpha) - alpha = target.
pub fn inv_inverse(target: f64) -> f64 {
    if target <= 0.0 {
        return 0.0;
    }
    // Initial guess from series inv(a) ~ a^3 / 3
    let mut a = libm::cbrt(3.0 * target).clamp(0.01, 1.45);
    for _ in 0..16 {
        let t = libm::tan(a);
        let f = t - a - target;
        let df = t * t; // d/da (tan(a) - a) = sec^2(a) - 1 = tan^2(a)
        if df < 1e-12 || libm::fabs(f) < 1e-12 {
            break;
        }
        a = (a - f / df).clamp(0.001, 1.5);
    }
    a
}

#[derive(Clone, Copy)]
pub struct MaterialProps {
    pub label: &'static str,
    /// Allowable bending stress (MPa) for preliminary Lewis check.
    pub sigma_b_allow_mpa: f64,
    /// Allowable contact stress (MPa) for preliminary Hertz check.
    pub sigma_h_allow_mpa: f64,
    /// Young's modulus (MPa).
    pub e_mpa: f64,
    /// Poisson's ratio.
    pub nu: f64,
}

pub fn material_props(id: &str) -> MaterialProps {
    match id {
        "steel_4140" => MaterialProps {
            label: "Alloy Steel 4140 QT",
            sigma_b_allow_mpa: 280.0,
            sigma_h_allow_mpa: 950.0,
            e_mpa: 205_000.0,
            nu: 0.29,
        },
        "steel_1045" => MaterialProps {
            label: "Carbon Steel 1045",
            sigma_b_allow_mpa: 190.0,
            sigma_h_allow_mpa: 650.0,
            e_mpa: 205_000.0,
            nu: 0.29,
        },
        "brass_c360" => MaterialProps {
            label: "Brass C360",
            sigma_b_allow_mpa: 95.0,
            sigma_h_allow_mpa: 350.0,
            e_mpa: 97_000.0,
            nu: 0.31,
        },
        "pom_delrin" => MaterialProps {
            label: "POM (Delrin / Acetal)",
            sigma_b_allow_mpa: 28.0,
            sigma_h_allow_mpa: 75.0,
            e_mpa: 2_900.0,
            nu: 0.35,
        },
        "pa66_nylon" => MaterialProps {
            label: "PA66 Nylon",
            sigma_b_allow_mpa: 24.0,
            sigma_h_allow_mpa: 65.0,
            e_mpa: 2_600.0,
            nu: 0.39,
        },
        "peek" => MaterialProps {
            label: "PEEK",
            sigma_b_allow_mpa: 45.0,
            sigma_h_allow_mpa: 110.0,
            e_mpa: 3_800.0,
            nu: 0.38,
        },
        "petg_3d" => MaterialProps {
            label: "3D-Printed PETG / PLA",
            sigma_b_allow_mpa: 14.0,
            sigma_h_allow_mpa: 40.0,
            e_mpa: 2_100.0,
            nu: 0.36,
        },
        _ => MaterialProps {
            label: "Carbon Steel 1045",
            sigma_b_allow_mpa: 190.0,
            sigma_h_allow_mpa: 650.0,
            e_mpa: 205_000.0,
            nu: 0.29,
        },
    }
}

/// Approximate Lewis form factor Y for 20-deg full-depth tooth as a function of virtual tooth count.
pub fn lewis_form_factor(z_virtual: f64, alpha_deg: f64) -> f64 {
    let zv = z_virtual.max(10.0);
    // Standard Lewis approximation: Y20 ~ 0.484 - 2.87 / zv
    let base_20 = 0.484 - 2.87 / zv;
    let factor = if alpha_deg < 17.0 {
        0.84
    } else if alpha_deg > 22.5 {
        1.14
    } else {
        1.0
    };
    (base_20 * factor).max(0.15)
}

pub struct MeshSolution {
    pub operating_pressure_angle_deg: f64,
    pub standard_center_distance_mm: f64,
    pub operating_center_distance_mm: f64,
    pub tip_shortening_mm: f64,
    pub contact_ratio_transverse: f64,
    pub contact_ratio_overlap: f64,
    pub contact_ratio_total: f64,
    pub z1_min_no_undercut: f64,
    pub x1_min_recommended: f64,
    pub face_width_ratio: f64,
    pub warnings: Vec<String>,
    pub strength: Option<Value>,
}

/// Solve exact ISO 21771 external gear pair mesh + preliminary Lewis/Hertz strength checks.
#[allow(clippy::too_many_arguments)]
pub fn solve_external_pair(
    m_n: f64,
    m_t: f64,
    z1: u32,
    z2: u32,
    alpha_n_deg: f64,
    alpha_t_deg: f64,
    beta_deg: f64,
    x1: f64,
    x2: f64,
    face_width: f64,
    backlash_cd_add_mm: f64,
    material_id: &str,
    input_torque_nm: f64,
    input_rpm: f64,
) -> MeshSolution {
    let deg2rad = core::f64::consts::PI / 180.0;
    let alpha_n = alpha_n_deg * deg2rad;
    let alpha_t = alpha_t_deg * deg2rad;
    let beta = beta_deg * deg2rad;
    let cos_b = libm::cos(beta);
    let sin_b = libm::sin(beta);

    let z1f = z1 as f64;
    let z2f = z2 as f64;
    let a_std = m_t * (z1f + z2f) / 2.0;

    // ISO 21771 operating pressure angle & center distance with profile shifts x1 + x2
    let inv_aw = inv(alpha_t) + 2.0 * (x1 + x2) * libm::tan(alpha_n) / (z1f + z2f);
    let alpha_w = if libm::fabs(x1 + x2) < 1e-9 {
        alpha_t
    } else {
        inv_inverse(inv_aw)
    };
    let a_w = a_std * (libm::cos(alpha_t) / libm::cos(alpha_w)) + backlash_cd_add_mm;
    let y_shift = (a_w - a_std) / m_n;
    let tip_shortening_mm = ((x1 + x2) - y_shift).max(0.0) * m_n;

    // Radii for transverse contact ratio epsilon_alpha
    let rp1 = m_t * z1f / 2.0;
    let rp2 = m_t * z2f / 2.0;
    let rb1 = rp1 * libm::cos(alpha_t);
    let rb2 = rp2 * libm::cos(alpha_t);
    let ra1 = rp1 + m_n * (1.0 + x1) - tip_shortening_mm;
    let ra2 = rp2 + m_n * (1.0 + x2) - tip_shortening_mm;
    let pb = core::f64::consts::PI * m_t * libm::cos(alpha_t);
    let g_alpha = libm::sqrt((ra1 * ra1 - rb1 * rb1).max(0.0))
        + libm::sqrt((ra2 * ra2 - rb2 * rb2).max(0.0))
        - a_w * libm::sin(alpha_w);
    let eps_alpha = (g_alpha / pb).max(0.0);
    let eps_beta = if beta_deg > 0.1 {
        (face_width * sin_b) / (core::f64::consts::PI * m_n)
    } else {
        0.0
    };
    let eps_total = eps_alpha + eps_beta;

    // Undercut check using virtual tooth count zv = z / cos^3(beta)
    let cos_b3 = cos_b * cos_b * cos_b;
    let z_min_base = 2.0 / (libm::sin(alpha_n) * libm::sin(alpha_n));
    let z1_min = z_min_base * cos_b3;
    let zv1 = z1f / cos_b3;
    let x1_min = (1.0 - zv1 / z_min_base).max(0.0);
    let fw_ratio = face_width / m_n;

    let mut warnings = Vec::new();
    if x1 < x1_min - 0.01 {
        warnings.push(format!(
            "Pinion undercut risk: Z1={} < {:.1} at {:.1} deg (recommend profile_shift_1 >= {:.2})",
            z1, z1_min, alpha_n_deg, x1_min
        ));
    }
    if eps_alpha < 1.2 {
        warnings.push(format!(
            "Low transverse contact ratio: {:.2} (target >= 1.4, minimum 1.2)",
            eps_alpha
        ));
    }
    if fw_ratio < 5.0 || fw_ratio > 14.0 {
        warnings.push(format!(
            "Face width ratio b/m = {:.1} outside typical 6..12 range",
            fw_ratio
        ));
    }

    // Optional preliminary Lewis bending + Hertzian contact stress check
    let strength = if input_torque_nm > 0.0 {
        let mat = material_props(material_id);
        let d1_mm = 2.0 * rp1;
        let ft_n = 2000.0 * input_torque_nm / d1_mm;
        let y_lewis = lewis_form_factor(zv1, alpha_n_deg);
        // Velocity factor Kv (Barth estimate) if RPM provided
        let v_mps = if input_rpm > 0.0 {
            core::f64::consts::PI * d1_mm * input_rpm / 60_000.0
        } else {
            0.0
        };
        let kv = if v_mps > 0.0 { (6.1 + v_mps) / 6.1 } else { 1.0 };
        let sigma_b = (ft_n * kv) / (face_width * m_n * y_lewis);
        let sf_b = mat.sigma_b_allow_mpa / sigma_b.max(1e-6);

        // Hertzian contact stress (ISO 6336 simplified):
        // sigma_H = Z_E * Z_H * sqrt(Ft * Kv / (b * d1) * (u + 1) / u)
        let u = z2f / z1f;
        let ze = libm::sqrt(1.0 / (core::f64::consts::PI * 2.0 * (1.0 - mat.nu * mat.nu) / mat.e_mpa));
        let zh = libm::sqrt(2.0 * cos_b / (libm::cos(alpha_t) * libm::sin(alpha_w)).max(0.05));
        let sigma_h = ze * zh * libm::sqrt((ft_n * kv / (face_width * d1_mm)) * ((u + 1.0) / u));
        let sf_h = mat.sigma_h_allow_mpa / sigma_h.max(1e-6);

        if sf_b < 1.2 {
            let rec_m = next_iso54_module(m_n * libm::sqrt(1.5 / sf_b.max(0.05)));
            warnings.push(format!(
                "Preliminary Lewis bending SF = {:.2} < 1.2 for {} (consider module >= {} mm or wider face)",
                sf_b, mat.label, rec_m
            ));
        }
        if sf_h < 1.1 {
            warnings.push(format!(
                "Preliminary Hertz contact SF = {:.2} < 1.1 for {} (pitting risk)",
                sf_h, mat.label
            ));
        }

        Some(json!({
            "note": "Preliminary Lewis bending & Hertzian contact estimate (not full ISO 6336 certification)",
            "material": mat.label,
            "tangential_load_n": ft_n,
            "pitch_line_velocity_mps": v_mps,
            "lewis_bending_mpa": sigma_b,
            "bending_safety_factor": sf_b,
            "hertz_contact_mpa": sigma_h,
            "contact_safety_factor": sf_h,
            "nearest_iso54_module_mm": nearest_iso54_module(m_n),
        }))
    } else {
        None
    };

    MeshSolution {
        operating_pressure_angle_deg: alpha_w / deg2rad,
        standard_center_distance_mm: a_std,
        operating_center_distance_mm: a_w,
        tip_shortening_mm,
        contact_ratio_transverse: eps_alpha,
        contact_ratio_overlap: eps_beta,
        contact_ratio_total: eps_total,
        z1_min_no_undercut: z1_min,
        x1_min_recommended: x1_min,
        face_width_ratio: fw_ratio,
        warnings,
        strength,
    }
}

impl MeshSolution {
    pub fn to_json(&self) -> Value {
        json!({
            "operating_pressure_angle_deg": self.operating_pressure_angle_deg,
            "standard_center_distance_mm": self.standard_center_distance_mm,
            "operating_center_distance_mm": self.operating_center_distance_mm,
            "tip_shortening_mm": self.tip_shortening_mm,
            "contact_ratio_transverse": self.contact_ratio_transverse,
            "contact_ratio_overlap": self.contact_ratio_overlap,
            "contact_ratio_total": self.contact_ratio_total,
            "z1_min_no_undercut": self.z1_min_no_undercut,
            "x1_min_recommended": self.x1_min_recommended,
            "face_width_ratio_b_over_m": self.face_width_ratio,
            "warnings": self.warnings,
            "strength_estimate": self.strength,
        })
    }
}

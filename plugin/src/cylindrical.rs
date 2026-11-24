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

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ToothKind {
    Straight,
    Helical,
    Herringbone,
}

impl ToothKind {
    pub fn parse(s: &str) -> Self {
        match s {
            "helical" => Self::Helical,
            "herringbone" => Self::Herringbone,
            _ => Self::Straight,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum GearSide {
    External,
    Internal,
}

impl GearSide {
    pub fn parse(s: &str) -> Self {
        match s {
            "internal" => Self::Internal,
            _ => Self::External,
        }
    }
}

pub struct GearSpec<'a> {
    pub derived_prefix: &'a str,
    pub m_param: &'a str,
    pub mt_param: &'a str,
    pub at_param: &'a str,
    pub z_param: &'a str,
    pub fw_param: &'a str,
    pub x_param: &'a str,
    pub bl_param: &'a str,
    pub bore_param: Option<&'a str>,
    pub rim_param: Option<&'a str>,
    pub phi_param: Option<&'a str>,
    pub gear_side: GearSide,
    pub tooth_kind: ToothKind,
    pub beta_param: Option<&'a str>,
    pub hand_sign: f64,
    pub frame: PlacementFrame,
    pub offset_x_param: Option<&'a str>,
    pub sketch_name: &'a str,
    pub path_sketch_name: &'a str,
    pub body_name: &'a str,
    pub m_val: f64,
    pub mt_val: f64,
    pub at_deg: f64,
    pub teeth_val: u32,
    pub shift_val: f64,
    pub bore_val: f64,
    pub rim_val: f64,
}

pub fn setup_transverse_params(
    existing: &mut BTreeSet<String>,
    p: &str,
    m_p: &str,
    a_p: &str,
    tooth_kind: ToothKind,
    helix_angle: f64,
    module_system: &str,
    m_val: f64,
    alpha_deg: f64,
) -> Result<(String, String, Option<String>, f64, f64), PluginError> {
    if tooth_kind == ToothKind::Straight {
        return Ok((m_p.to_string(), a_p.to_string(), None, m_val, alpha_deg));
    }
    if !(1.0..=45.0).contains(&helix_angle) {
        return Err(invalid("helix_angle", "helix_angle must be between 1 and 45 deg"));
    }
    let beta_p = format!("{p}_beta");
    upsert_param(existing, &beta_p, &fmt_num(helix_angle), "deg", "Helix angle", true)?;
    let beta_rad = helix_angle * core::f64::consts::PI / 180.0;
    let cos_b = libm::cos(beta_rad);

    if module_system == "transverse" {
        Ok((m_p.to_string(), a_p.to_string(), Some(beta_p), m_val, alpha_deg))
    } else {
        let mt_p = format!("{p}_mt");
        let at_p = format!("{p}_at");
        upsert_param(existing, &mt_p, &format!("{m_p} / cos({beta_p})"), "mm", "Transverse module", false)?;
        upsert_param(
            existing,
            &at_p,
            &format!("atan(tan({a_p}) / cos({beta_p}))"),
            "deg",
            "Transverse pressure angle",
            false,
        )?;
        let mt_val = m_val / cos_b;
        let alpha_rad = alpha_deg * core::f64::consts::PI / 180.0;
        let at_deg = libm::atan(libm::tan(alpha_rad) / cos_b) * 180.0 / core::f64::consts::PI;
        Ok((mt_p, at_p, Some(beta_p), mt_val, at_deg))
    }
}

pub fn build_parametric_gear(
    existing: &mut BTreeSet<String>,
    spec: &GearSpec<'_>,
) -> Result<Value, PluginError> {
    let gp = spec.derived_prefix;
    let m = spec.m_param;
    let mt = spec.mt_param;
    let at = spec.at_param;
    let z = spec.z_param;
    let fw = spec.fw_param;
    let x = spec.x_param;
    let bl = spec.bl_param;
    let is_internal = spec.gear_side == GearSide::Internal;

    let rp = format!("{gp}_rp");
    let rb = format!("{gp}_rb");
    let ra = format!("{gp}_ra");
    let rf = format!("{gp}_rf");
    let r_start = format!("{gp}_r_start");
    let t_start = format!("{gp}_t_start");
    let t_end = format!("{gp}_t_end");
    let inv_a = format!("{gp}_inv_a");
    let psi = format!("{gp}_psi");
    let hp = format!("{gp}_hp");

    let pitch_dia = spec.mt_val * spec.teeth_val as f64;
    let at_rad = spec.at_deg * core::f64::consts::PI / 180.0;
    let base_dia = pitch_dia * libm::cos(at_rad);
    // For internal ring gear, tooth tips point inward (r_inner = rp - 1.0*m) and root is outward (r_outer_tooth = rp + 1.25*m).
    // We still draw the exact involute space cutout between r_in and r_out and surround it with an outer ring circle!
    let (r_in_expr, r_out_expr, r_in_val, r_out_val) = if is_internal {
        (
            format!("{rp} - {m} * (1 - {x})"),
            format!("{rp} + {m} * (1.25 + {x})"),
            pitch_dia / 2.0 - spec.m_val * (1.0 - spec.shift_val),
            pitch_dia / 2.0 + spec.m_val * (1.25 + spec.shift_val),
        )
    } else {
        (
            format!("{rp} - {m} * (1.25 - {x})"),
            format!("{rp} + {m} * (1 + {x})"),
            pitch_dia / 2.0 - spec.m_val * (1.25 - spec.shift_val),
            pitch_dia / 2.0 + spec.m_val * (1.0 + spec.shift_val),
        )
    };

    if r_in_val <= 0.25 {
        return Err(invalid("teeth", "inner/root radius is too small for this module/shift"));
    }
    if !is_internal && spec.bore_val > 0.0 && spec.bore_val >= 2.0 * r_in_val - 0.5 {
        return Err(invalid(
            "bore_diameter",
            format!("bore ({:.2} mm) must be smaller than root diameter ({:.2} mm)", spec.bore_val, 2.0 * r_in_val),
        ));
    }

    upsert_param(existing, &rp, &format!("{mt} * {z} / 2"), "mm", "Pitch radius", false)?;
    upsert_param(existing, &rb, &format!("{rp} * cos({at})"), "mm", "Base radius", false)?;
    upsert_param(existing, &ra, &r_out_expr, "mm", "Outer tooth radius", false)?;
    upsert_param(existing, &rf, &r_in_expr, "mm", "Inner tooth radius", false)?;
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
        "Involute outer roll parameter (rad)",
        false,
    )?;
    upsert_param(
        existing,
        &inv_a,
        &format!("tan({at}) - {at} / 1 rad"),
        "",
        "Involute function at transverse pressure angle",
        false,
    )?;
    // For an internal gear, the cutout is the tooth *space* (widened by backlash +bl/mt so the mating external tooth has clearance).
    let psi_expr = if is_internal {
        format!("(pi / 2 + 2 * {x} * tan({at}) + {bl} / {mt}) / {z} + {inv_a}")
    } else {
        format!("(pi / 2 + 2 * {x} * tan({at}) - {bl} / {mt}) / {z} + {inv_a}")
    };
    upsert_param(existing, &psi, &psi_expr, "", "Half tooth/space angular width on base circle (rad)", false)?;
    upsert_param(existing, &hp, &format!("pi / {z}"), "", "Half tooth pitch angle (rad)", false)?;

    let is_herringbone = spec.tooth_kind == ToothKind::Herringbone;
    let build_h = if is_herringbone {
        let fh = format!("{gp}_fw_half");
        upsert_param(existing, &fh, &format!("{fw} / 2"), "mm", "Half face width (herringbone)", false)?;
        fh
    } else {
        fw.to_string()
    };

    let base_origin = spec.frame.origin_json(spec.offset_x_param);
    let axis_vec = json!([spec.frame.z_dir[0], spec.frame.z_dir[1], spec.frame.z_dir[2]]);

    // 1. Complete 2D gear cross-section sketch.
    run_cmd(
        "sketch.create",
        json!({
            "plane": spec.frame.plane_json(base_origin.clone()),
            "name": spec.sketch_name,
        }),
    )?;

    // Draw outer ring circle (internal gear) or bore circle (external gear) BEFORE circular pattern.
    let ring_outer_dia = if is_internal {
        let rim_p = spec.rim_param.unwrap_or("6 mm");
        let ro_p = format!("{gp}_ring_od");
        upsert_param(existing, &ro_p, &format!("2 * ({ra} + {rim_p})"), "mm", "Internal ring outer diameter", false)?;
        run_cmd(
            "sketch.circle.center",
            json!({ "center": [0.0, 0.0], "diameter": ro_p }),
        )?;
        2.0 * (r_out_val + spec.rim_val)
    } else {
        if let Some(bore_p) = spec.bore_param {
            if spec.bore_val > 0.0 {
                run_cmd(
                    "sketch.circle.center",
                    json!({ "center": [0.0, 0.0], "diameter": bore_p }),
                )?;
            }
        }
        2.0 * r_out_val
    };

    let (phi_prefix, x_bot, y_bot, x_top, y_top, tip_mid) = match spec.phi_param {
        Some(phi) => (
            format!("{phi} + "),
            format!("{rb} * (cos({phi} - {psi} + t) + t * sin({phi} - {psi} + t))"),
            format!("{rb} * (sin({phi} - {psi} + t) - t * cos({phi} - {psi} + t))"),
            format!("{rb} * (cos({phi} + {psi} - t) - t * sin({phi} + {psi} - t))"),
            format!("{rb} * (sin({phi} + {psi} - t) + t * cos({phi} + {psi} - t))"),
            json!([
                format!("{ra} * cos({phi} * 1 rad)"),
                format!("{ra} * sin({phi} * 1 rad)")
            ]),
        ),
        None => (
            String::new(),
            format!("{rb} * (cos(t - {psi}) + t * sin(t - {psi}))"),
            format!("{rb} * (sin(t - {psi}) - t * cos(t - {psi}))"),
            format!("{rb} * (cos({psi} - t) - t * sin({psi} - t))"),
            format!("{rb} * (sin({psi} - t) + t * cos({psi} - t))"),
            json!([ra, 0.0]),
        ),
    };

    let bot_res = run_cmd(
        "sketch.equation_curve",
        json!({ "x": x_bot, "y": y_bot, "start": t_start, "end": t_end, "tolerance": 0.02 }),
    )?;
    let bot_curve = extract_created_curve_id(&bot_res, "bottom flank")?;
    let (bot_start, bot_end) = extract_curve_endpoints(&bot_res, "bottom flank")?;

    let top_res = run_cmd(
        "sketch.equation_curve",
        json!({ "x": x_top, "y": y_top, "start": t_start, "end": t_end, "tolerance": 0.02 }),
    )?;
    let top_curve = extract_created_curve_id(&top_res, "top flank")?;
    let (top_start, top_end) = extract_curve_endpoints(&top_res, "top flank")?;

    let tip_res = run_cmd(
        "sketch.arc.three_point",
        json!({ "start": bot_end, "through": tip_mid, "end": top_end, "connect": true }),
    )?;
    let tip_curve = extract_created_curve_id(&tip_res, "outer arc")?;

    let ang = |expr: String| format!("({phi_prefix}{expr}) * 1 rad");
    let pt_on_rf = |a_expr: String| -> Value {
        json!([format!("{rf} * cos({a_expr})"), format!("{rf} * sin({a_expr})")])
    };
    let p_hp_neg = pt_on_rf(ang(format!("-{hp}")));
    let p_hp_pos = pt_on_rf(ang(hp.clone()));

    let mut pattern_entities: Vec<String> = Vec::new();
    pattern_entities.push(bot_curve);
    pattern_entities.push(top_curve);
    pattern_entities.push(tip_curve);

    let needs_radial = r_in_val < base_dia / 2.0 - 1e-3;
    if needs_radial {
        let p_psi_neg = pt_on_rf(ang(format!("-{psi}")));
        let p_psi_pos = pt_on_rf(ang(psi.clone()));
        let p_mid_neg = pt_on_rf(ang(format!("-0.5 * ({hp} + {psi})")));
        let p_mid_pos = pt_on_rf(ang(format!("0.5 * ({hp} + {psi})")));

        let a_bot = run_cmd(
            "sketch.arc.three_point",
            json!({ "start": p_hp_neg, "through": p_mid_neg, "end": p_psi_neg, "connect": true }),
        )?;
        let l_bot = run_cmd(
            "sketch.line",
            json!({ "points": [p_psi_neg, bot_start], "connect": true }),
        )?;
        let l_top = run_cmd(
            "sketch.line",
            json!({ "points": [top_start, p_psi_pos], "connect": true }),
        )?;
        let a_top = run_cmd(
            "sketch.arc.three_point",
            json!({ "start": p_psi_pos, "through": p_mid_pos, "end": p_hp_pos, "connect": true }),
        )?;
        pattern_entities.push(extract_created_curve_id(&a_bot, "bot inner arc")?);
        pattern_entities.push(extract_created_curve_id(&l_bot, "bot radial line")?);
        pattern_entities.push(extract_created_curve_id(&l_top, "top radial line")?);
        pattern_entities.push(extract_created_curve_id(&a_top, "top inner arc")?);
    } else {
        let psi_f = format!("{gp}_psi_f");
        upsert_param(
            existing,
            &psi_f,
            &format!("{psi} - ({t_start} - atan({t_start}) / 1 rad)"),
            "",
            "Half angular width on inner circle (rad)",
            false,
        )?;
        let p_mid_neg = pt_on_rf(ang(format!("-0.5 * ({hp} + {psi_f})")));
        let p_mid_pos = pt_on_rf(ang(format!("0.5 * ({hp} + {psi_f})")));
        let a_bot = run_cmd(
            "sketch.arc.three_point",
            json!({ "start": p_hp_neg, "through": p_mid_neg, "end": bot_start, "connect": true }),
        )?;
        let a_top = run_cmd(
            "sketch.arc.three_point",
            json!({ "start": top_start, "through": p_mid_pos, "end": p_hp_pos, "connect": true }),
        )?;
        pattern_entities.push(extract_created_curve_id(&a_bot, "bot inner arc")?);
        pattern_entities.push(extract_created_curve_id(&a_top, "top inner arc")?);
    }

    run_cmd(
        "sketch.pattern.circular",
        json!({ "entities": pattern_entities, "center": [0.0, 0.0], "count": z, "angle": "360 deg" }),
    )?;
    run_cmd("sketch.finish", json!({}))?;

    // For internal ring gear, pick the annular ring profile (the one with a hole!).
    // For external gear, pick the largest-area profile (the gear body, not the bore hole).
    let gear_profile = pick_outer_profile_index(spec.sketch_name, is_internal)?;

    let actual_body = if spec.tooth_kind == ToothKind::Straight {
        let ext_res = run_cmd(
            "solid.extrude",
            json!({
                "sketch": spec.sketch_name,
                "profiles": [gear_profile],
                "distance": &build_h,
                "operation": "new_body",
                "body_name": spec.body_name,
            }),
        )?;
        ext_res
            .get("created_bodies")
            .and_then(Value::as_array)
            .and_then(|a| a.first())
            .and_then(Value::as_str)
            .unwrap_or(spec.body_name)
            .to_string()
    } else {
        let beta = spec.beta_param.unwrap_or("15 deg");
        let twist_p = format!("{gp}_twist");
        let sign = if spec.hand_sign < 0.0 { "-" } else { "" };
        upsert_param(
            existing,
            &twist_p,
            &format!("{sign}({build_h} * tan({beta}) / {rp}) * (180 / pi)"),
            "deg",
            "Helical sweep twist angle",
            false,
        )?;

        run_cmd(
            "sketch.create",
            json!({
                "plane": {
                    "origin": base_origin,
                    "x_dir": [spec.frame.x_dir[0], spec.frame.x_dir[1], spec.frame.x_dir[2]],
                    "y_dir": [spec.frame.z_dir[0], spec.frame.z_dir[1], spec.frame.z_dir[2]],
                },
                "name": spec.path_sketch_name,
            }),
        )?;
        let line_res = run_cmd("sketch.line", json!({ "points": [[0.0, 0.0], [0.0, &build_h]] }))?;
        let path_curve = extract_created_curve_id(&line_res, "helical axis path")?;
        run_cmd("sketch.finish", json!({}))?;

        let sweep_res = run_cmd(
            "solid.sweep",
            json!({
                "sketch": spec.sketch_name,
                "profiles": [gear_profile],
                "path_sketch": spec.path_sketch_name,
                "path": [path_curve],
                "twist": twist_p,
                "operation": "new_body",
                "body_name": spec.body_name,
            }),
        )?;
        sweep_res
            .get("created_bodies")
            .and_then(Value::as_array)
            .and_then(|a| a.first())
            .and_then(Value::as_str)
            .unwrap_or(spec.body_name)
            .to_string()
    };

    if is_herringbone {
        let mid_origin = spec.frame.origin_offset_json(spec.offset_x_param, Some(&build_h));
        run_cmd(
            "solid.mirror",
            json!({
                "bodies": [&actual_body],
                "combine": true,
                "plane": { "origin": mid_origin, "normal": axis_vec },
            }),
        )?;
    }

    Ok(json!({
        "body": actual_body,
        "sketch": spec.sketch_name,
        "pitch_diameter_mm": pitch_dia,
        "outer_diameter_mm": ring_outer_dia,
        "root_diameter_mm": 2.0 * r_in_val,
    }))
}

#[derive(Deserialize)]
#[serde(default)]
struct CylindricalGearInput {
    placement: Vec<Value>,
    axis_name: String,
    base_plane: String,
    flip_direction: bool,
    axial_offset: f64,
    gear_side: String,
    tooth_type: String,
    helix_angle: f64,
    hand: String,
    module_system: String,
    prefix: String,
    module: f64,
    teeth: u32,
    pressure_angle: f64,
    face_width: f64,
    bore_diameter: f64,
    rim_thickness: f64,
    profile_shift: f64,
    backlash: f64,
    body_name: String,
}

impl Default for CylindricalGearInput {
    fn default() -> Self {
        Self {
            placement: Vec::new(),
            axis_name: String::new(),
            base_plane: "XY".into(),
            flip_direction: false,
            axial_offset: 0.0,
            gear_side: "external".into(),
            tooth_type: "straight".into(),
            helix_angle: 15.0,
            hand: "right".into(),
            module_system: "normal".into(),
            prefix: "g1".into(),
            module: 2.0,
            teeth: 20,
            pressure_angle: 20.0,
            face_width: 10.0,
            bore_diameter: 8.0,
            rim_thickness: 6.0,
            profile_shift: 0.0,
            backlash: 0.05,
            body_name: "GM_Gear".into(),
        }
    }
}

pub fn run_cylindrical_gear(raw: &Value) -> Result<String, PluginError> {
    let inp: CylindricalGearInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    if inp.module <= 0.0 || inp.face_width <= 0.0 {
        return Err(invalid("module", "module and face_width must be positive"));
    }
    if inp.teeth < 8 || inp.teeth > 120 {
        return Err(invalid("teeth", "teeth must be between 8 and 120"));
    }
    let gear_side = GearSide::parse(&inp.gear_side);
    let tooth_kind = ToothKind::parse(&inp.tooth_type);
    let hand_sign = if inp.hand == "left" { -1.0 } else { 1.0 };

    let frame = resolve_placement(
        &inp.placement,
        &inp.axis_name,
        &inp.base_plane,
        inp.flip_direction,
        inp.axial_offset,
        0.0,
    )?;
    let mut existing = list_existing_params()?;
    let mut existing_names = list_existing_doc_names()?;
    let p = choose_unique_prefix(&p, &existing, &existing_names);

    let m_p = format!("{p}_m");
    let z_p = format!("{p}_z");
    let a_p = format!("{p}_alpha");
    let fw_p = format!("{p}_fw");
    let x_p = format!("{p}_x");
    let bl_p = format!("{p}_bl");
    let bore_p = format!("{p}_bore");
    let rim_p = format!("{p}_rim");

    upsert_param(&mut existing, &m_p, &fmt_num(inp.module), "mm", "Gear module", true)?;
    upsert_param(&mut existing, &z_p, &format!("{}", inp.teeth), "", "Tooth count", true)?;
    upsert_param(&mut existing, &a_p, &fmt_num(inp.pressure_angle), "deg", "Pressure angle", true)?;
    upsert_param(&mut existing, &fw_p, &fmt_num(inp.face_width), "mm", "Face width", true)?;
    upsert_param(&mut existing, &x_p, &fmt_num(inp.profile_shift), "", "Profile shift coefficient", false)?;
    upsert_param(&mut existing, &bl_p, &fmt_num(inp.backlash), "mm", "Backlash", false)?;
    if gear_side == GearSide::Internal {
        if inp.rim_thickness <= 0.5 {
            return Err(invalid("rim_thickness", "rim thickness must be > 0.5 mm"));
        }
        upsert_param(&mut existing, &rim_p, &fmt_num(inp.rim_thickness), "mm", "Ring wall thickness", true)?;
    } else if inp.bore_diameter > 0.0 {
        upsert_param(&mut existing, &bore_p, &fmt_num(inp.bore_diameter), "mm", "Bore diameter", true)?;
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

    let sketch_name = unique_name(&format!("{p}_ToothSketch"), &mut existing_names);
    let path_sketch_name = unique_name(&format!("{p}_AxisPath"), &mut existing_names);
    let desired_body = if inp.body_name.trim().is_empty() {
        format!("{p}_Gear")
    } else {
        inp.body_name.trim().to_string()
    };
    let body_name = unique_name(&desired_body, &mut existing_names);

    let info = build_parametric_gear(
        &mut existing,
        &GearSpec {
            derived_prefix: &p,
            m_param: &m_p,
            mt_param: &mt_p,
            at_param: &at_p,
            z_param: &z_p,
            fw_param: &fw_p,
            x_param: &x_p,
            bl_param: &bl_p,
            bore_param: if gear_side == GearSide::External && inp.bore_diameter > 0.0 {
                Some(&bore_p)
            } else {
                None
            },
            rim_param: if gear_side == GearSide::Internal { Some(&rim_p) } else { None },
            phi_param: None,
            gear_side,
            tooth_kind,
            beta_param: beta_p.as_deref(),
            hand_sign,
            frame,
            offset_x_param: None,
            sketch_name: &sketch_name,
            path_sketch_name: &path_sketch_name,
            body_name: &body_name,
            m_val: inp.module,
            mt_val,
            at_deg,
            teeth_val: inp.teeth,
            shift_val: inp.profile_shift,
            bore_val: inp.bore_diameter,
            rim_val: inp.rim_thickness,
        },
    )?;

    Ok(json!({
        "mode": "gm-cylindrical-gear",
        "gear_side": inp.gear_side,
        "tooth_type": inp.tooth_type,
        "prefix": p,
        "gear": info,
    })
    .to_string())
}

#[derive(Deserialize)]
#[serde(default)]
struct ParallelPairInput {
    placement: Vec<Value>,
    axis_name: String,
    base_plane: String,
    layout_angle: f64,
    flip_direction: bool,
    axial_offset: f64,
    tooth_type: String,
    helix_angle: f64,
    hand: String,
    module_system: String,
    prefix: String,
    module: f64,
    teeth_1: u32,
    teeth_2: u32,
    pressure_angle: f64,
    face_width: f64,
    bore_1: f64,
    bore_2: f64,
    backlash: f64,
    backlash_mode: String,
}

impl Default for ParallelPairInput {
    fn default() -> Self {
        Self {
            placement: Vec::new(),
            axis_name: String::new(),
            base_plane: "XY".into(),
            layout_angle: 0.0,
            flip_direction: false,
            axial_offset: 0.0,
            tooth_type: "straight".into(),
            helix_angle: 15.0,
            hand: "right".into(),
            module_system: "normal".into(),
            prefix: "p1".into(),
            module: 2.0,
            teeth_1: 16,
            teeth_2: 32,
            pressure_angle: 20.0,
            face_width: 10.0,
            bore_1: 6.0,
            bore_2: 10.0,
            backlash: 0.05,
            backlash_mode: "tooth_thinning".into(),
        }
    }
}

pub fn run_parallel_pair(raw: &Value) -> Result<String, PluginError> {
    let inp: ParallelPairInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    if inp.module <= 0.0 || inp.face_width <= 0.0 {
        return Err(invalid("module", "module and face_width must be positive"));
    }
    if inp.teeth_1 < 8 || inp.teeth_1 > 100 || inp.teeth_2 < 8 || inp.teeth_2 > 100 {
        return Err(invalid("teeth_1", "tooth counts must be between 8 and 100"));
    }
    let tooth_kind = ToothKind::parse(&inp.tooth_type);
    let hand_sign_1 = if inp.hand == "left" { -1.0 } else { 1.0 };
    let hand_sign_2 = -hand_sign_1;

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
    let z1_p = format!("{p}_z1");
    let z2_p = format!("{p}_z2");
    let a_p = format!("{p}_alpha");
    let fw_p = format!("{p}_fw");
    let x0_p = format!("{p}_x");
    let bl_p = format!("{p}_bl");
    let bl_tooth_p = format!("{p}_bl_tooth");
    let b1_p = format!("{p}_bore1");
    let b2_p = format!("{p}_bore2");
    let cd_p = format!("{p}_cd");
    let ratio_p = format!("{p}_ratio");
    let phi2_p = format!("{p}_g2_phi");

    let cd_backlash = inp.backlash_mode == "center_distance";

    upsert_param(&mut existing, &m_p, &fmt_num(inp.module), "mm", "Shared module", true)?;
    upsert_param(&mut existing, &z1_p, &format!("{}", inp.teeth_1), "", "Pinion tooth count (Z1)", true)?;
    upsert_param(&mut existing, &z2_p, &format!("{}", inp.teeth_2), "", "Gear tooth count (Z2)", true)?;
    upsert_param(&mut existing, &a_p, &fmt_num(inp.pressure_angle), "deg", "Shared pressure angle", true)?;
    upsert_param(&mut existing, &fw_p, &fmt_num(inp.face_width), "mm", "Shared face width", true)?;
    upsert_param(&mut existing, &x0_p, "0", "", "Profile shift coefficient", false)?;
    upsert_param(&mut existing, &bl_p, &fmt_num(inp.backlash), "mm", "Total circumferential meshing backlash jt", true)?;
    if cd_backlash {
        upsert_param(&mut existing, &bl_tooth_p, "0", "mm", "Per-gear tooth thinning (0 in center-distance mode)", false)?;
    } else {
        upsert_param(&mut existing, &bl_tooth_p, &format!("{bl_p} / 2"), "mm", "Per-gear tooth thinning (jt / 2)", false)?;
    }
    if inp.bore_1 > 0.0 {
        upsert_param(&mut existing, &b1_p, &fmt_num(inp.bore_1), "mm", "Pinion bore diameter", true)?;
    }
    if inp.bore_2 > 0.0 {
        upsert_param(&mut existing, &b2_p, &fmt_num(inp.bore_2), "mm", "Gear bore diameter", true)?;
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

    let cd_expr = if cd_backlash {
        format!("{mt_p} * ({z1_p} + {z2_p}) / 2 + {bl_p} / (2 * tan({at_p}))")
    } else {
        format!("{mt_p} * ({z1_p} + {z2_p}) / 2")
    };
    upsert_param(&mut existing, &cd_p, &cd_expr, "mm", "ISO 21771 shaft center distance", true)?;
    upsert_param(&mut existing, &ratio_p, &format!("{z2_p} / {z1_p}"), "", "Gear ratio Z2 / Z1", true)?;
    upsert_param(&mut existing, &phi2_p, &format!("pi + pi / {z2_p}"), "", "Gear 2 meshing phase angle (rad)", false)?;

    let g1_prefix = format!("{p}_g1");
    let g1_sketch = unique_name(&format!("{p}_PinionTooth"), &mut existing_names);
    let g1_path = unique_name(&format!("{p}_PinionAxis"), &mut existing_names);
    let g1_body = unique_name(&format!("{p}_Pinion"), &mut existing_names);
    let g1 = build_parametric_gear(
        &mut existing,
        &GearSpec {
            derived_prefix: &g1_prefix,
            m_param: &m_p,
            mt_param: &mt_p,
            at_param: &at_p,
            z_param: &z1_p,
            fw_param: &fw_p,
            x_param: &x0_p,
            bl_param: &bl_tooth_p,
            bore_param: if inp.bore_1 > 0.0 { Some(&b1_p) } else { None },
            rim_param: None,
            phi_param: None,
            gear_side: GearSide::External,
            tooth_kind,
            beta_param: beta_p.as_deref(),
            hand_sign: hand_sign_1,
            frame,
            offset_x_param: None,
            sketch_name: &g1_sketch,
            path_sketch_name: &g1_path,
            body_name: &g1_body,
            m_val: inp.module,
            mt_val,
            at_deg,
            teeth_val: inp.teeth_1,
            shift_val: 0.0,
            bore_val: inp.bore_1,
            rim_val: 0.0,
        },
    )?;

    let g2_prefix = format!("{p}_g2");
    let g2_sketch = unique_name(&format!("{p}_GearTooth"), &mut existing_names);
    let g2_path = unique_name(&format!("{p}_GearAxis"), &mut existing_names);
    let g2_body = unique_name(&format!("{p}_Gear"), &mut existing_names);
    let g2 = build_parametric_gear(
        &mut existing,
        &GearSpec {
            derived_prefix: &g2_prefix,
            m_param: &m_p,
            mt_param: &mt_p,
            at_param: &at_p,
            z_param: &z2_p,
            fw_param: &fw_p,
            x_param: &x0_p,
            bl_param: &bl_tooth_p,
            bore_param: if inp.bore_2 > 0.0 { Some(&b2_p) } else { None },
            rim_param: None,
            phi_param: Some(&phi2_p),
            gear_side: GearSide::External,
            tooth_kind,
            beta_param: beta_p.as_deref(),
            hand_sign: hand_sign_2,
            frame,
            offset_x_param: Some(&cd_p),
            sketch_name: &g2_sketch,
            path_sketch_name: &g2_path,
            body_name: &g2_body,
            m_val: inp.module,
            mt_val,
            at_deg,
            teeth_val: inp.teeth_2,
            shift_val: 0.0,
            bore_val: inp.bore_2,
            rim_val: 0.0,
        },
    )?;

    let at_rad = at_deg * core::f64::consts::PI / 180.0;
    let cd_mm = mt_val * (inp.teeth_1 + inp.teeth_2) as f64 / 2.0
        + if cd_backlash { inp.backlash / (2.0 * libm::tan(at_rad)) } else { 0.0 };
    Ok(json!({
        "mode": "gm-parallel-pair",
        "tooth_type": inp.tooth_type,
        "prefix": p,
        "center_distance_mm": cd_mm,
        "gear_ratio": inp.teeth_2 as f64 / inp.teeth_1 as f64,
        "pinion": g1,
        "gear": g2,
    })
    .to_string())
}

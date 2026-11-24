use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use serde::Deserialize;
use serde_json::{Value, json};

use crate::cylindrical::{GearSide, GearSpec, ToothKind, build_parametric_gear, setup_transverse_params};
use crate::placement::resolve_placement;
use crate::{
    PluginError, choose_unique_prefix, fmt_num, invalid, list_existing_doc_names,
    list_existing_params, unique_name, upsert_param, validate_prefix,
};

#[derive(Deserialize)]
#[serde(default)]
struct PlanetaryInput {
    placement: Vec<Value>,
    axis_name: String,
    base_plane: String,
    flip_direction: bool,
    axial_offset: f64,
    tooth_type: String,
    helix_angle: f64,
    hand: String,
    prefix: String,
    module: f64,
    sun_teeth: u32,
    planet_teeth: u32,
    num_planets: u32,
    pressure_angle: f64,
    face_width: f64,
    sun_bore: f64,
    planet_bore: f64,
    ring_wall: f64,
    backlash: f64,
}

impl Default for PlanetaryInput {
    fn default() -> Self {
        Self {
            placement: Vec::new(),
            axis_name: String::new(),
            base_plane: "XY".into(),
            flip_direction: false,
            axial_offset: 0.0,
            tooth_type: "straight".into(),
            helix_angle: 15.0,
            hand: "right".into(),
            prefix: "pl1".into(),
            module: 2.0,
            sun_teeth: 18,
            planet_teeth: 12,
            num_planets: 3,
            pressure_angle: 20.0,
            face_width: 10.0,
            sun_bore: 6.0,
            planet_bore: 5.0,
            ring_wall: 6.0,
            backlash: 0.08,
        }
    }
}

pub fn run_planetary(raw: &Value) -> Result<String, PluginError> {
    let inp: PlanetaryInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    if inp.module <= 0.0 || inp.face_width <= 0.0 {
        return Err(invalid("module", "module and face_width must be positive"));
    }
    if inp.sun_teeth < 8 || inp.planet_teeth < 8 {
        return Err(invalid("sun_teeth", "sun and planet teeth must be at least 8"));
    }
    if inp.num_planets < 2 || inp.num_planets > 8 {
        return Err(invalid("num_planets", "num_planets must be between 2 and 8"));
    }
    let ring_teeth = inp.sun_teeth + 2 * inp.planet_teeth;
    if (inp.sun_teeth + ring_teeth) % inp.num_planets != 0 {
        return Err(invalid(
            "num_planets",
            format!(
                "Planetary assembly rule failed: (Z_sun + Z_ring) = ({} + {}) = {} is not divisible by {} planets",
                inp.sun_teeth,
                ring_teeth,
                inp.sun_teeth + ring_teeth,
                inp.num_planets
            ),
        ));
    }

    let tooth_kind = ToothKind::parse(&inp.tooth_type);
    let sun_hand = if inp.hand == "left" { -1.0 } else { 1.0 };
    let planet_hand = -sun_hand;
    // Internal ring meshes externally with planets -> same hand as planets!
    let ring_hand = planet_hand;

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
    let zs_p = format!("{p}_zs");
    let zp_p = format!("{p}_zp");
    let zr_p = format!("{p}_zr");
    let a_p = format!("{p}_alpha");
    let fw_p = format!("{p}_fw");
    let x0_p = format!("{p}_x");
    let bl_p = format!("{p}_bl");
    let bl_half_p = format!("{p}_bl_half");
    let bs_p = format!("{p}_sun_bore");
    let bp_p = format!("{p}_planet_bore");
    let rw_p = format!("{p}_ring_wall");
    let orbit_p = format!("{p}_orbit_r");
    let ratio_p = format!("{p}_ratio");
    let phi_pl_p = format!("{p}_pl_phi");
    let phi_ring_p = format!("{p}_ring_phi");

    upsert_param(&mut existing, &m_p, &fmt_num(inp.module), "mm", "Planetary module", true)?;
    upsert_param(&mut existing, &zs_p, &format!("{}", inp.sun_teeth), "", "Sun teeth", true)?;
    upsert_param(&mut existing, &zp_p, &format!("{}", inp.planet_teeth), "", "Planet teeth", true)?;
    upsert_param(&mut existing, &zr_p, &format!("{zs_p} + 2 * {zp_p}"), "", "Ring teeth (Zs + 2*Zp)", true)?;
    upsert_param(&mut existing, &a_p, &fmt_num(inp.pressure_angle), "deg", "Pressure angle", true)?;
    upsert_param(&mut existing, &fw_p, &fmt_num(inp.face_width), "mm", "Face width", true)?;
    upsert_param(&mut existing, &x0_p, "0", "", "Profile shift", false)?;
    upsert_param(&mut existing, &bl_p, &fmt_num(inp.backlash), "mm", "Total meshing backlash", true)?;
    upsert_param(&mut existing, &bl_half_p, &format!("{bl_p} / 2"), "mm", "Half backlash per gear", false)?;
    if inp.sun_bore > 0.0 {
        upsert_param(&mut existing, &bs_p, &fmt_num(inp.sun_bore), "mm", "Sun bore diameter", true)?;
    }
    if inp.planet_bore > 0.0 {
        upsert_param(&mut existing, &bp_p, &fmt_num(inp.planet_bore), "mm", "Planet bore diameter", true)?;
    }
    upsert_param(&mut existing, &rw_p, &fmt_num(inp.ring_wall), "mm", "Ring wall thickness", true)?;

    let (mt_p, at_p, beta_p, mt_val, at_deg) = setup_transverse_params(
        &mut existing,
        &p,
        &m_p,
        &a_p,
        tooth_kind,
        inp.helix_angle,
        "normal",
        inp.module,
        inp.pressure_angle,
    )?;

    upsert_param(&mut existing, &orbit_p, &format!("{mt_p} * ({zs_p} + {zp_p}) / 2"), "mm", "Planet carrier orbit radius", true)?;
    upsert_param(&mut existing, &ratio_p, &format!("1 + {zr_p} / {zs_p}"), "", "Reduction ratio (ring fixed)", true)?;
    // Planet at +X meshes with Sun's tooth at +X -> planet space at pi -> tooth at pi + pi/Zp.
    upsert_param(&mut existing, &phi_pl_p, &format!("pi + pi / {zp_p}"), "", "Planet meshing phase (rad)", false)?;
    // At +X, if Zp is even, planet has a tooth space at 0 -> ring needs a tooth at 0 (space at pi/Zr).
    // If Zp is odd, planet has a tooth at 0 -> ring needs a space at 0!
    let ring_phase = if inp.planet_teeth % 2 == 0 {
        format!("pi / {zr_p}")
    } else {
        "0".to_string()
    };
    upsert_param(&mut existing, &phi_ring_p, &ring_phase, "", "Ring meshing phase (rad)", false)?;

    // 1. Sun gear
    let sun_sk = unique_name(&format!("{p}_SunSketch"), &mut existing_names);
    let sun_path = unique_name(&format!("{p}_SunAxis"), &mut existing_names);
    let sun_body = unique_name(&format!("{p}_Sun"), &mut existing_names);
    let sun = build_parametric_gear(
        &mut existing,
        &GearSpec {
            derived_prefix: &format!("{p}_sun"),
            m_param: &m_p,
            mt_param: &mt_p,
            at_param: &at_p,
            z_param: &zs_p,
            fw_param: &fw_p,
            x_param: &x0_p,
            bl_param: &bl_half_p,
            bore_param: if inp.sun_bore > 0.0 { Some(&bs_p) } else { None },
            rim_param: None,
            phi_param: None,
            gear_side: GearSide::External,
            tooth_kind,
            beta_param: beta_p.as_deref(),
            hand_sign: sun_hand,
            frame,
            offset_x_param: None,
            sketch_name: &sun_sk,
            path_sketch_name: &sun_path,
            body_name: &sun_body,
            m_val: inp.module,
            mt_val,
            at_deg,
            teeth_val: inp.sun_teeth,
            shift_val: 0.0,
            bore_val: inp.sun_bore,
            rim_val: 0.0,
        },
    )?;

    // 2. N Planet gears around orbit radius
    let mut planets = Vec::new();
    for k in 0..inp.num_planets {
        let ang_deg = 360.0 * (k as f64) / (inp.num_planets as f64);
        let p_frame = frame.rotated_z(ang_deg);
        let pl_sk = unique_name(&format!("{p}_Planet{}_Sketch", k + 1), &mut existing_names);
        let pl_path = unique_name(&format!("{p}_Planet{}_Axis", k + 1), &mut existing_names);
        let pl_body = unique_name(&format!("{p}_Planet{}", k + 1), &mut existing_names);
        // Compute exact phase for planet k so it meshes cleanly with sun tooth orientation at angle ang_deg.
        let sun_teeth_at_k = (inp.sun_teeth as f64) * (k as f64) / (inp.num_planets as f64);
        let frac = sun_teeth_at_k - libm::floor(sun_teeth_at_k);
        let pl_phi_k = format!("{p}_pl{}_phi", k + 1);
        let phi_expr = if libm::fabs(frac) < 1e-6 {
            phi_pl_p.clone()
        } else {
            format!("pi + pi / {zp_p} - {} * (2 * pi / {zp_p})", fmt_num(frac))
        };
        upsert_param(&mut existing, &pl_phi_k, &phi_expr, "", "Planet k phase", false)?;

        let pl = build_parametric_gear(
            &mut existing,
            &GearSpec {
                derived_prefix: &format!("{p}_pl{}", k + 1),
                m_param: &m_p,
                mt_param: &mt_p,
                at_param: &at_p,
                z_param: &zp_p,
                fw_param: &fw_p,
                x_param: &x0_p,
                bl_param: &bl_half_p,
                bore_param: if inp.planet_bore > 0.0 { Some(&bp_p) } else { None },
                rim_param: None,
                phi_param: Some(&pl_phi_k),
                gear_side: GearSide::External,
                tooth_kind,
                beta_param: beta_p.as_deref(),
                hand_sign: planet_hand,
                frame: p_frame,
                offset_x_param: Some(&orbit_p),
                sketch_name: &pl_sk,
                path_sketch_name: &pl_path,
                body_name: &pl_body,
                m_val: inp.module,
                mt_val,
                at_deg,
                teeth_val: inp.planet_teeth,
                shift_val: 0.0,
                bore_val: inp.planet_bore,
                rim_val: 0.0,
            },
        )?;
        planets.push(pl);
    }

    // 3. Internal Ring gear
    let ring_sk = unique_name(&format!("{p}_RingSketch"), &mut existing_names);
    let ring_path = unique_name(&format!("{p}_RingAxis"), &mut existing_names);
    let ring_body = unique_name(&format!("{p}_Ring"), &mut existing_names);
    let ring = build_parametric_gear(
        &mut existing,
        &GearSpec {
            derived_prefix: &format!("{p}_ring"),
            m_param: &m_p,
            mt_param: &mt_p,
            at_param: &at_p,
            z_param: &zr_p,
            fw_param: &fw_p,
            x_param: &x0_p,
            bl_param: &bl_half_p,
            bore_param: None,
            rim_param: Some(&rw_p),
            phi_param: if inp.planet_teeth % 2 == 0 { Some(&phi_ring_p) } else { None },
            gear_side: GearSide::Internal,
            tooth_kind,
            beta_param: beta_p.as_deref(),
            hand_sign: ring_hand,
            frame,
            offset_x_param: None,
            sketch_name: &ring_sk,
            path_sketch_name: &ring_path,
            body_name: &ring_body,
            m_val: inp.module,
            mt_val,
            at_deg,
            teeth_val: ring_teeth,
            shift_val: 0.0,
            bore_val: 0.0,
            rim_val: inp.ring_wall,
        },
    )?;

    Ok(json!({
        "mode": "gm-planetary",
        "prefix": p,
        "ring_teeth": ring_teeth,
        "orbit_radius_mm": mt_val * (inp.sun_teeth + inp.planet_teeth) as f64 / 2.0,
        "reduction_ratio": 1.0 + (ring_teeth as f64 / inp.sun_teeth as f64),
        "sun": sun,
        "planets": planets,
        "ring": ring,
    })
    .to_string())
}

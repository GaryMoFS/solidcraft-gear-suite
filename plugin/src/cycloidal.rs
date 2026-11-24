use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use serde::Deserialize;
use serde_json::{Value, json};

use crate::placement::resolve_placement;
use crate::{
    PluginError, choose_unique_prefix, fmt_num, invalid, list_existing_doc_names,
    list_existing_params, pick_outer_profile_index, run_cmd, unique_name, upsert_param,
    validate_prefix,
};

#[derive(Deserialize)]
#[serde(default)]
struct CycloidalInput {
    placement: Vec<Value>,
    axis_name: String,
    base_plane: String,
    flip_direction: bool,
    axial_offset: f64,
    prefix: String,
    num_lobes: u32,
    pin_circle_dia: f64,
    pin_dia: f64,
    eccentricity: f64,
    disc_thickness: f64,
    bore_dia: f64,
    output_hole_count: u32,
    output_hole_dia: f64,
    output_pcd: f64,
    include_pins: bool,
}

impl Default for CycloidalInput {
    fn default() -> Self {
        Self {
            placement: Vec::new(),
            axis_name: String::new(),
            base_plane: "XY".into(),
            flip_direction: false,
            axial_offset: 0.0,
            prefix: "cyc1".into(),
            num_lobes: 11,
            pin_circle_dia: 80.0,
            pin_dia: 8.0,
            eccentricity: 1.5,
            disc_thickness: 10.0,
            bore_dia: 16.0,
            output_hole_count: 6,
            output_hole_dia: 10.0,
            output_pcd: 44.0,
            include_pins: true,
        }
    }
}

pub fn run_cycloidal(raw: &Value) -> Result<String, PluginError> {
    let inp: CycloidalInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    if inp.num_lobes < 4 || inp.num_lobes > 40 {
        return Err(invalid("num_lobes", "num_lobes must be between 4 and 40"));
    }
    let n_pins = inp.num_lobes + 1;
    let r_big = inp.pin_circle_dia / 2.0;
    let r_pin = inp.pin_dia / 2.0;
    if inp.eccentricity <= 0.0 || inp.eccentricity >= r_big / (n_pins as f64) * 0.95 {
        return Err(invalid(
            "eccentricity",
            format!("eccentricity must be > 0 and < R / (N+1) = {:.2} mm to avoid cusps", r_big / (n_pins as f64)),
        ));
    }
    if inp.disc_thickness <= 0.5 {
        return Err(invalid("disc_thickness", "disc_thickness must be > 0.5 mm"));
    }

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

    let r_p = format!("{p}_R");
    let rr_p = format!("{p}_Rr");
    let e_p = format!("{p}_e");
    let n_p = format!("{p}_N");
    let np_p = format!("{p}_Npins");
    let th_p = format!("{p}_th");
    let bore_p = format!("{p}_bore");

    upsert_param(&mut existing, &r_p, &fmt_num(r_big), "mm", "Pin pitch circle radius", true)?;
    upsert_param(&mut existing, &rr_p, &fmt_num(r_pin), "mm", "Roller pin radius", true)?;
    upsert_param(&mut existing, &e_p, &fmt_num(inp.eccentricity), "mm", "Eccentricity", true)?;
    upsert_param(&mut existing, &n_p, &format!("{}", inp.num_lobes), "", "Cycloidal lobes (reduction ratio N:1)", true)?;
    upsert_param(&mut existing, &np_p, &format!("{n_p} + 1"), "", "Stationary ring pin count (N+1)", true)?;
    upsert_param(&mut existing, &th_p, &fmt_num(inp.disc_thickness), "mm", "Cycloidal disc thickness", true)?;
    if inp.bore_dia > 0.0 {
        upsert_param(&mut existing, &bore_p, &fmt_num(inp.bore_dia), "mm", "Center bearing bore", true)?;
    }

    // 1. Closed parametric equidistant epitrochoid disc sketch.
    let disc_sk = unique_name(&format!("{p}_DiscSketch"), &mut existing_names);
    let disc_body = unique_name(&format!("{p}_CycloidalDisc"), &mut existing_names);
    let base_origin = frame.origin_json(None);

    run_cmd(
        "sketch.create",
        json!({ "plane": frame.plane_json(base_origin.clone()), "name": disc_sk }),
    )?;

    // Center bearing bore
    if inp.bore_dia > 0.0 {
        run_cmd("sketch.circle.center", json!({ "center": [0.0, 0.0], "diameter": &bore_p }))?;
    }
    // Output pin holes
    if inp.output_hole_count > 0 && inp.output_hole_dia > 0.0 && inp.output_pcd > inp.bore_dia {
        let r_out = inp.output_pcd / 2.0;
        for k in 0..inp.output_hole_count {
            let ang = 2.0 * core::f64::consts::PI * (k as f64) / (inp.output_hole_count as f64);
            run_cmd(
                "sketch.circle.center",
                json!({
                    "center": [r_out * libm::cos(ang), r_out * libm::sin(ang)],
                    "diameter": inp.output_hole_dia,
                }),
            )?;
        }
    }

    // Equidistant epitrochoid formula:
    // psi(t) = atan2(sin(N*t), R/(e*Npins) - cos(N*t)) / 1 rad
    // x(t) = R*cos(t) - Rr*cos(t + psi(t)) - e*cos(Npins*t)
    // y(t) = -R*sin(t) + Rr*sin(t + psi(t)) + e*sin(Npins*t)
    let psi_expr = format!("atan2(sin({n_p} * t), {r_p} / ({e_p} * {np_p}) - cos({n_p} * t)) / 1 rad");
    let x_eq = format!("{r_p} * cos(t) - {rr_p} * cos(t + ({psi_expr})) - {e_p} * cos({np_p} * t)");
    let y_eq = format!("-{r_p} * sin(t) + {rr_p} * sin(t + ({psi_expr})) + {e_p} * sin({np_p} * t)");
    run_cmd(
        "sketch.equation_curve",
        json!({
            "x": x_eq,
            "y": y_eq,
            "start": 0.0,
            "end": "2 * pi",
            "tolerance": 0.02,
        }),
    )?;
    run_cmd("sketch.finish", json!({}))?;

    let disc_prof = pick_outer_profile_index(&disc_sk, false)?;
    run_cmd(
        "solid.extrude",
        json!({
            "sketch": disc_sk,
            "profiles": [disc_prof],
            "distance": &th_p,
            "operation": "new_body",
            "body_name": disc_body,
        }),
    )?;

    // 2. Optional stationary ring pins (offset by +e along +X so disc sits in meshed eccentric position!).
    let mut pins_body: Option<String> = None;
    if inp.include_pins {
        let pins_sk = unique_name(&format!("{p}_PinsSketch"), &mut existing_names);
        let pb = unique_name(&format!("{p}_RingPins"), &mut existing_names);
        run_cmd(
            "sketch.create",
            json!({ "plane": frame.plane_json(base_origin), "name": pins_sk }),
        )?;
        for k in 0..n_pins {
            let ang = 2.0 * core::f64::consts::PI * (k as f64) / (n_pins as f64);
            let cx = inp.eccentricity + r_big * libm::cos(ang);
            let cy = r_big * libm::sin(ang);
            run_cmd(
                "sketch.circle.center",
                json!({ "center": [cx, cy], "diameter": inp.pin_dia }),
            )?;
        }
        run_cmd("sketch.finish", json!({}))?;
        run_cmd(
            "solid.extrude",
            json!({
                "sketch": pins_sk,
                "distance": &th_p,
                "operation": "new_body",
                "body_name": pb,
            }),
        )?;
        pins_body = Some(pb);
    }

    Ok(json!({
        "mode": "gm-cycloidal",
        "prefix": p,
        "reduction_ratio": inp.num_lobes,
        "ring_pin_count": n_pins,
        "disc_body": disc_body,
        "pins_body": pins_body,
    })
    .to_string())
}

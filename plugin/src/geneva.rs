use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use serde::Deserialize;
use serde_json::{Value, json};

use crate::placement::resolve_placement;
use crate::{
    PluginError, choose_unique_prefix, extract_created_curve_id, fmt_num, invalid,
    list_existing_doc_names, list_existing_params, pick_outer_profile_index, run_cmd, unique_name,
    upsert_param, validate_prefix,
};

#[derive(Deserialize)]
#[serde(default)]
struct GenevaInput {
    placement: Vec<Value>,
    axis_name: String,
    base_plane: String,
    layout_angle: f64,
    flip_direction: bool,
    axial_offset: f64,
    prefix: String,
    num_slots: u32,
    center_distance: f64,
    pin_dia: f64,
    thickness: f64,
    clearance: f64,
    wheel_bore: f64,
    driver_bore: f64,
    include_driver: bool,
}

impl Default for GenevaInput {
    fn default() -> Self {
        Self {
            placement: Vec::new(),
            axis_name: String::new(),
            base_plane: "XY".into(),
            layout_angle: 0.0,
            flip_direction: false,
            axial_offset: 0.0,
            prefix: "gen1".into(),
            num_slots: 6,
            center_distance: 60.0,
            pin_dia: 6.0,
            thickness: 8.0,
            clearance: 0.2,
            wheel_bore: 10.0,
            driver_bore: 8.0,
            include_driver: true,
        }
    }
}

pub fn run_geneva(raw: &Value) -> Result<String, PluginError> {
    let inp: GenevaInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    if inp.num_slots < 3 || inp.num_slots > 12 {
        return Err(invalid("num_slots", "num_slots must be between 3 and 12"));
    }
    if inp.center_distance < 15.0 || inp.pin_dia <= 0.5 || inp.thickness <= 0.5 {
        return Err(invalid("center_distance", "center_distance (>=15), pin_dia, and thickness must be positive"));
    }

    let n = inp.num_slots as f64;
    let c = inp.center_distance;
    let alpha = core::f64::consts::PI / n;
    let r_crank = c * libm::sin(alpha);
    let r_wheel = c * libm::cos(alpha);
    let w_half = inp.pin_dia / 2.0 + inp.clearance;
    // Inner slot center radius (where drive pin bottoms out at mid-index)
    let r_slot_in = (c - r_crank).max(inp.wheel_bore / 2.0 + w_half + 1.5);
    if r_slot_in >= r_wheel - 1.5 {
        return Err(invalid("center_distance", "center_distance too small for pin_dia and wheel_bore"));
    }

    // Locking crescent radius on driver (and matching concave cutout on wheel between slots)
    let r_lock_max = r_crank - inp.pin_dia - inp.clearance - 1.0;
    let r_lock = r_lock_max.max(inp.driver_bore / 2.0 + 2.0);
    let r_cut = r_lock + inp.clearance;

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

    let th_p = format!("{p}_th");
    let cd_p = format!("{p}_cd");
    upsert_param(&mut existing, &cd_p, &fmt_num(c), "mm", "Geneva center distance", true)?;
    upsert_param(&mut existing, &th_p, &fmt_num(inp.thickness), "mm", "Geneva wheel thickness", true)?;
    upsert_param(&mut existing, &format!("{p}_slots"), &format!("{}", inp.num_slots), "", "Number of Geneva slots", true)?;
    upsert_param(&mut existing, &format!("{p}_crank_r"), &fmt_num(r_crank), "mm", "Drive crank pin radius", true)?;
    upsert_param(&mut existing, &format!("{p}_wheel_r"), &fmt_num(r_wheel), "mm", "Geneva wheel outer radius", true)?;

    // 1. Build Geneva Wheel in ONE fast 2D sketch (1 sector patterned N times + bore).
    let wheel_sk = unique_name(&format!("{p}_WheelSketch"), &mut existing_names);
    let wheel_body = unique_name(&format!("{p}_GenevaWheel"), &mut existing_names);
    let base_origin = frame.origin_json(None);

    run_cmd(
        "sketch.create",
        json!({ "plane": frame.plane_json(base_origin.clone()), "name": wheel_sk }),
    )?;
    if inp.wheel_bore > 0.0 {
        run_cmd("sketch.circle.center", json!({ "center": [0.0, 0.0], "diameter": inp.wheel_bore }))?;
    }

    // Slot 0 lies along +X. Slot walls at y = +/- w_half meet outer circle r_wheel at angle +/- phi_slot.
    let phi_slot = libm::asin((w_half / r_wheel).clamp(0.01, 0.95));
    let x_rim = r_wheel * libm::cos(phi_slot);

    // Check if locking crescent circle (radius r_cut centered at distance c along angle +/- alpha)
    // intersects the outer wheel circle r_wheel between phi_slot and alpha.
    // Law of cosines: cos(gamma) = (r_wheel^2 + c^2 - r_cut^2) / (2 * r_wheel * c)
    let cos_g = ((r_wheel * r_wheel + c * c - r_cut * r_cut) / (2.0 * r_wheel * c)).clamp(-1.0, 1.0);
    let gamma = libm::acos(cos_g);
    let has_crescent = r_lock > 2.0 && (alpha - gamma) > phi_slot + 0.02 && (c - r_cut) < r_wheel - 0.2;

    let mut entities: Vec<String> = Vec::new();
    // Slot inner U-bottom semicircle centered at (r_slot_in, 0), radius w_half
    let p_in_bot = json!([r_slot_in, -w_half]);
    let p_in_mid = json!([r_slot_in - w_half, 0.0]);
    let p_in_top = json!([r_slot_in, w_half]);
    let p_rim_bot = json!([x_rim, -w_half]);
    let p_rim_top = json!([x_rim, w_half]);

    let l_bot = run_cmd("sketch.line", json!({ "points": [p_rim_bot, p_in_bot], "connect": true }))?;
    let a_in = run_cmd("sketch.arc.three_point", json!({ "start": p_in_bot, "through": p_in_mid, "end": p_in_top, "connect": true }))?;
    let l_top = run_cmd("sketch.line", json!({ "points": [p_in_top, p_rim_top], "connect": true }))?;
    entities.push(extract_created_curve_id(&l_bot, "slot bot")?);
    entities.push(extract_created_curve_id(&a_in, "slot bottom arc")?);
    entities.push(extract_created_curve_id(&l_top, "slot top")?);

    if has_crescent {
        let ang_join = alpha - gamma;
        let p_j_neg = json!([r_wheel * libm::cos(-ang_join), r_wheel * libm::sin(-ang_join)]);
        let p_j_pos = json!([r_wheel * libm::cos(ang_join), r_wheel * libm::sin(ang_join)]);
        let p_rm_neg = json!([r_wheel * libm::cos(-0.5 * (phi_slot + ang_join)), r_wheel * libm::sin(-0.5 * (phi_slot + ang_join))]);
        let p_rm_pos = json!([r_wheel * libm::cos(0.5 * (phi_slot + ang_join)), r_wheel * libm::sin(0.5 * (phi_slot + ang_join))]);

        // Crescent midpoint at +/- alpha, radius (c - r_cut)
        let r_cresc = c - r_cut;
        let p_cr_neg = json!([r_cresc * libm::cos(-alpha), r_cresc * libm::sin(-alpha)]);
        let p_cr_pos = json!([r_cresc * libm::cos(alpha), r_cresc * libm::sin(alpha)]);

        // Midpoints on the concave locking arcs centered at (c*cos(+-alpha), c*sin(+-alpha))
        let cx_pos = c * libm::cos(alpha);
        let cy_pos = c * libm::sin(alpha);
        let a_start_pos = libm::atan2(r_wheel * libm::sin(ang_join) - cy_pos, r_wheel * libm::cos(ang_join) - cx_pos);
        let a_end_pos = alpha + core::f64::consts::PI;
        let a_mid_pos = 0.5 * (a_start_pos + a_end_pos);
        // Pick the angle closer to the wheel side
        let pm_pos_x = cx_pos + r_cut * libm::cos(a_mid_pos);
        let pm_pos_y = cy_pos + r_cut * libm::sin(a_mid_pos);
        let (cm_pos, cm_neg) = if pm_pos_x * pm_pos_x + pm_pos_y * pm_pos_y < r_wheel * r_wheel {
            (json!([pm_pos_x, pm_pos_y]), json!([pm_pos_x, -pm_pos_y]))
        } else {
            let alt = a_mid_pos + core::f64::consts::PI;
            let ax = cx_pos + r_cut * libm::cos(alt);
            let ay = cy_pos + r_cut * libm::sin(alt);
            (json!([ax, ay]), json!([ax, -ay]))
        };

        let a_cr_neg = run_cmd("sketch.arc.three_point", json!({ "start": p_cr_neg, "through": cm_neg, "end": p_j_neg, "connect": true }))?;
        let a_rim_neg = run_cmd("sketch.arc.three_point", json!({ "start": p_j_neg, "through": p_rm_neg, "end": p_rim_bot, "connect": true }))?;
        let a_rim_pos = run_cmd("sketch.arc.three_point", json!({ "start": p_rim_top, "through": p_rm_pos, "end": p_j_pos, "connect": true }))?;
        let a_cr_pos = run_cmd("sketch.arc.three_point", json!({ "start": p_j_pos, "through": cm_pos, "end": p_cr_pos, "connect": true }))?;
        entities.push(extract_created_curve_id(&a_cr_neg, "crescent neg")?);
        entities.push(extract_created_curve_id(&a_rim_neg, "rim neg")?);
        entities.push(extract_created_curve_id(&a_rim_pos, "rim pos")?);
        entities.push(extract_created_curve_id(&a_cr_pos, "crescent pos")?);
    } else {
        let p_a_neg = json!([r_wheel * libm::cos(-alpha), r_wheel * libm::sin(-alpha)]);
        let p_a_pos = json!([r_wheel * libm::cos(alpha), r_wheel * libm::sin(alpha)]);
        let p_m_neg = json!([r_wheel * libm::cos(-0.5 * (phi_slot + alpha)), r_wheel * libm::sin(-0.5 * (phi_slot + alpha))]);
        let p_m_pos = json!([r_wheel * libm::cos(0.5 * (phi_slot + alpha)), r_wheel * libm::sin(0.5 * (phi_slot + alpha))]);
        let a_neg = run_cmd("sketch.arc.three_point", json!({ "start": p_a_neg, "through": p_m_neg, "end": p_rim_bot, "connect": true }))?;
        let a_pos = run_cmd("sketch.arc.three_point", json!({ "start": p_rim_top, "through": p_m_pos, "end": p_a_pos, "connect": true }))?;
        entities.push(extract_created_curve_id(&a_neg, "rim neg")?);
        entities.push(extract_created_curve_id(&a_pos, "rim pos")?);
    }

    run_cmd(
        "sketch.pattern.circular",
        json!({ "entities": entities, "center": [0.0, 0.0], "count": inp.num_slots, "angle": "360 deg" }),
    )?;
    run_cmd("sketch.finish", json!({}))?;
    let w_prof = pick_outer_profile_index(&wheel_sk, false)?;

    run_cmd(
        "solid.extrude",
        json!({
            "sketch": wheel_sk,
            "profiles": [w_prof],
            "distance": &th_p,
            "operation": "new_body",
            "body_name": wheel_body,
        }),
    )?;

    // 2. Optional Drive Crank + Drive Pin + Locking Disc at x = C!
    let mut driver_body: Option<String> = None;
    if inp.include_driver {
        let db = unique_name(&format!("{p}_GenevaDriver"), &mut existing_names);
        let base_h = (inp.thickness * 0.6).max(3.0);
        let driver_origin = frame.world_point(c, 0.0, -base_h);
        let axis_vec = [frame.z_dir[0], frame.z_dir[1], frame.z_dir[2]];
        let disc_r = r_crank + inp.pin_dia * 0.8;

        // Drive base disc sitting directly beneath the wheel plane
        run_cmd(
            "solid.cylinder",
            json!({
                "radius": disc_r,
                "height": base_h,
                "base": driver_origin,
                "axis": axis_vec,
                "operation": "new_body",
                "body_name": &db,
            }),
        )?;

        // Drive pin engaging Slot 0 at (c - r_crank, 0)
        let pin_origin = frame.world_point(c - r_crank, 0.0, 0.0);
        run_cmd(
            "solid.cylinder",
            json!({
                "diameter": inp.pin_dia,
                "height": inp.thickness,
                "base": pin_origin,
                "axis": axis_vec,
                "operation": "join",
                "targets": [&db],
            }),
        )?;

        // Locking hub crescent (if there's room): cylinder of radius r_lock on wheel plane, cut by wheel clearance on pin side
        if has_crescent {
            let lock_origin = frame.world_point(c, 0.0, 0.0);
            let cut_origin = frame.world_point(0.0, 0.0, -0.5);
            let lock_tmp = unique_name(&format!("{p}_LockTmp"), &mut existing_names);
            run_cmd(
                "solid.cylinder",
                json!({
                    "radius": r_lock,
                    "height": inp.thickness,
                    "base": lock_origin,
                    "axis": axis_vec,
                    "operation": "new_body",
                    "body_name": &lock_tmp,
                }),
            )?;
            run_cmd(
                "solid.cylinder",
                json!({
                    "radius": r_wheel + inp.clearance,
                    "height": inp.thickness + 1.0,
                    "base": cut_origin,
                    "axis": axis_vec,
                    "operation": "cut",
                    "targets": [&lock_tmp],
                }),
            )?;
            run_cmd(
                "solid.combine",
                json!({
                    "target": &db,
                    "tools": [&lock_tmp],
                    "operation": "join",
                    "keep_tools": false,
                }),
            )?;
        }

        if inp.driver_bore > 0.0 && inp.driver_bore < 2.0 * r_lock - 1.0 {
            let bore_origin = frame.world_point(c, 0.0, -base_h - 0.5);
            run_cmd(
                "solid.cylinder",
                json!({
                    "diameter": inp.driver_bore,
                    "height": base_h + inp.thickness + 1.0,
                    "base": bore_origin,
                    "axis": axis_vec,
                    "operation": "cut",
                    "targets": [&db],
                }),
            )?;
        }
        driver_body = Some(db);
    }

    Ok(json!({
        "mode": "gm-geneva",
        "prefix": p,
        "num_slots": inp.num_slots,
        "index_angle_deg": 360.0 / n,
        "crank_radius_mm": r_crank,
        "wheel_radius_mm": r_wheel,
        "wheel_body": wheel_body,
        "driver_body": driver_body,
    })
    .to_string())
}

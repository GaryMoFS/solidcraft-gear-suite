use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use serde::Deserialize;
use serde_json::{Value, json};

use crate::placement::{PlacementFrame, parse_vec3, resolve_placement};
use crate::{
    PluginError, extract_created_curve_id, fmt_num, invalid, list_existing_doc_names,
    pick_outer_profile_index, run_cmd, unique_name,
};

#[derive(Deserialize)]
#[serde(default)]
struct KnurlInput {
    target_face: Vec<Value>,
    base_plane: String,
    knurl_type: String,
    diameter: f64,
    length: f64,
    pitch: f64,
    depth: f64,
    angle: f64,
    bore_diameter: f64,
    body_name: String,
}

impl Default for KnurlInput {
    fn default() -> Self {
        Self {
            target_face: Vec::new(),
            base_plane: "XY".into(),
            knurl_type: "diamond".into(),
            diameter: 20.0,
            length: 15.0,
            pitch: 1.5,
            depth: 0.5,
            angle: 30.0,
            bore_diameter: 0.0,
            body_name: "GM_Knurl".into(),
        }
    }
}

fn build_knurl_pass(
    existing_names: &mut alloc::collections::BTreeSet<String>,
    frame: PlacementFrame,
    r_outer: f64,
    r_root: f64,
    bore_dia: f64,
    length: f64,
    teeth: u32,
    twist_deg: f64,
    tag: &str,
    body_name: &str,
) -> Result<String, PluginError> {
    let sk = unique_name(&format!("{tag}_Sketch"), existing_names);
    let bname = unique_name(body_name, existing_names);

    run_cmd(
        "sketch.create",
        json!({ "plane": frame.plane_json(frame.origin_json(None)), "name": sk }),
    )?;
    if bore_dia > 0.0 && bore_dia < 2.0 * r_root - 0.5 {
        run_cmd("sketch.circle.center", json!({ "center": [0.0, 0.0], "diameter": bore_dia }))?;
    }

    // 1 sawtooth V-profile segment spanning [-pi/N, +pi/N], patterned in 2D via sketch.pattern.circular!
    let hp = core::f64::consts::PI / (teeth as f64);
    let p0 = json!([r_root * libm::cos(-hp), r_root * libm::sin(-hp)]);
    let p1 = json!([r_outer, 0.0]);
    let p2 = json!([r_root * libm::cos(hp), r_root * libm::sin(hp)]);
    let l1 = run_cmd("sketch.line", json!({ "points": [p0, p1], "connect": true }))?;
    let l2 = run_cmd("sketch.line", json!({ "points": [p1, p2], "connect": true }))?;
    let c1 = extract_created_curve_id(&l1, "knurl flank 1")?;
    let c2 = extract_created_curve_id(&l2, "knurl flank 2")?;
    run_cmd(
        "sketch.pattern.circular",
        json!({ "entities": [c1, c2], "center": [0.0, 0.0], "count": teeth, "angle": "360 deg" }),
    )?;
    run_cmd("sketch.finish", json!({}))?;
    let prof = pick_outer_profile_index(&sk, false)?;

    let created = if libm::fabs(twist_deg) < 1e-4 {
        run_cmd(
            "solid.extrude",
            json!({
                "sketch": sk,
                "profiles": [prof],
                "distance": length,
                "operation": "new_body",
                "body_name": bname,
            }),
        )?
    } else {
        let psk = unique_name(&format!("{tag}_Axis"), existing_names);
        run_cmd(
            "sketch.create",
            json!({
                "plane": {
                    "origin": frame.origin_json(None),
                    "x_dir": [frame.x_dir[0], frame.x_dir[1], frame.x_dir[2]],
                    "y_dir": [frame.z_dir[0], frame.z_dir[1], frame.z_dir[2]],
                },
                "name": psk,
            }),
        )?;
        let lres = run_cmd("sketch.line", json!({ "points": [[0.0, 0.0], [0.0, length]] }))?;
        let pcurve = extract_created_curve_id(&lres, "knurl axis")?;
        run_cmd("sketch.finish", json!({}))?;
        run_cmd(
            "solid.sweep",
            json!({
                "sketch": sk,
                "profiles": [prof],
                "path_sketch": psk,
                "path": [pcurve],
                "twist": format!("{} deg", fmt_num(twist_deg)),
                "operation": "new_body",
                "body_name": bname,
            }),
        )?
    };
    Ok(created
        .get("created_bodies")
        .and_then(Value::as_array)
        .and_then(|a| a.first())
        .and_then(Value::as_str)
        .unwrap_or(&bname)
        .to_string())
}

pub fn run_knurl(raw: &Value) -> Result<String, PluginError> {
    let inp: KnurlInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    if inp.pitch <= 0.2 || inp.depth <= 0.05 {
        return Err(invalid("pitch", "pitch must be > 0.2 mm and depth > 0.05 mm"));
    }

    let mut frame = resolve_placement(&inp.target_face, "", &inp.base_plane, false, 0.0, 0.0)?;
    let mut diameter = inp.diameter;
    let length = inp.length;

    // If a cylindrical face was picked, automatically adopt its exact diameter, origin, and axis!
    if let Some(pick) = inp.target_face.first() {
        if pick.get("type").and_then(Value::as_str) == Some("face") {
            if let Some(body) = pick.get("body").and_then(Value::as_str) {
                if let Ok(faces_res) = run_cmd("model.faces", json!({ "body": body })) {
                    if let Some(faces) = faces_res.get("faces").and_then(Value::as_array) {
                        let idx = pick.get("index").and_then(Value::as_u64);
                        if let Some(face) = idx
                            .and_then(|i| faces.iter().find(|f| f.get("index").and_then(Value::as_u64) == Some(i)))
                            .or_else(|| faces.first())
                        {
                            if let Some(cyl) = face.get("cylinder").and_then(Value::as_object) {
                                if let Some(r) = cyl.get("radius").and_then(Value::as_f64) {
                                    diameter = 2.0 * r;
                                }
                                if let (Some(o), Some(a)) = (
                                    cyl.get("origin").and_then(parse_vec3),
                                    cyl.get("axis").and_then(parse_vec3),
                                ) {
                                    frame = PlacementFrame::from_axis(o, a);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    if diameter <= 2.0 * inp.depth + 1.0 || length <= 0.5 {
        return Err(invalid("diameter", "diameter must be larger than 2*depth and length > 0.5 mm"));
    }

    let r_outer = diameter / 2.0;
    let r_root = r_outer - inp.depth;
    let raw_n = libm::round(core::f64::consts::PI * diameter / inp.pitch) as u32;
    let teeth = raw_n.clamp(12, 72);
    let ang_rad = inp.angle.clamp(5.0, 60.0) * core::f64::consts::PI / 180.0;
    let twist_mag = (length * libm::tan(ang_rad) / r_outer) * 180.0 / core::f64::consts::PI;

    let mut existing_names = list_existing_doc_names()?;
    let desired = if inp.body_name.trim().is_empty() {
        "GM_Knurl"
    } else {
        inp.body_name.trim()
    };

    let final_body = match inp.knurl_type.as_str() {
        "straight" => build_knurl_pass(
            &mut existing_names,
            frame,
            r_outer,
            r_root,
            inp.bore_diameter,
            length,
            teeth,
            0.0,
            "kn",
            desired,
        )?,
        "diagonal" => build_knurl_pass(
            &mut existing_names,
            frame,
            r_outer,
            r_root,
            inp.bore_diameter,
            length,
            teeth,
            twist_mag,
            "kn",
            desired,
        )?,
        _ => {
            // Diamond knurl: intersect RH and LH twisted sawtooth cylinders -> true 3D diamond pyramids!
            let b_rh = build_knurl_pass(
                &mut existing_names,
                frame,
                r_outer,
                r_root,
                inp.bore_diameter,
                length,
                teeth,
                twist_mag,
                "kn_rh",
                desired,
            )?;
            let b_lh = build_knurl_pass(
                &mut existing_names,
                frame,
                r_outer,
                r_root,
                inp.bore_diameter,
                length,
                teeth,
                -twist_mag,
                "kn_lh",
                &format!("{desired}_LH"),
            )?;
            run_cmd(
                "solid.combine",
                json!({
                    "target": &b_rh,
                    "tools": [&b_lh],
                    "operation": "intersect",
                    "keep_tools": false,
                }),
            )?;
            b_rh
        }
    };

    Ok(json!({
        "mode": "gm-knurl",
        "knurl_type": inp.knurl_type,
        "diameter_mm": diameter,
        "teeth": teeth,
        "body": final_body,
    })
    .to_string())
}

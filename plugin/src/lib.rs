#![no_std]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

extern crate alloc;

use alloc::collections::BTreeSet;
use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use serde::Deserialize;
use serde_json::{Value, json};

#[global_allocator]
static ALLOCATOR: dlmalloc::GlobalDlmalloc = dlmalloc::GlobalDlmalloc;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    core::arch::wasm32::unreachable()
}

wit_bindgen::generate!({
    path: "../wit/plugin.wit",
    world: "command-plugin",
});

use craft::plugin::commands;
use craft::plugin::control;
use craft::plugin::types::{FieldFailure, PluginError};
use exports::craft::plugin::entry::Guest;

struct GearSuitePlugin;

export!(GearSuitePlugin);

impl Guest for GearSuitePlugin {
    fn execute(entry: String, params: String) -> Result<String, PluginError> {
        let value: Value =
            serde_json::from_str(&params).map_err(|e| invalid("params", format!("invalid JSON: {e}")))?;
        match entry.as_str() {
            "gm-cylindrical-gear" | "parametric-spur-gear" => run_cylindrical_gear(&value),
            "gm-parallel-pair" | "parametric-parallel-pair" => run_parallel_pair(&value),
            _ => Err(invalid("entry", format!("unknown entry `{entry}`"))),
        }
    }
}

fn invalid(field: &str, reason: impl Into<String>) -> PluginError {
    PluginError::InvalidInput(FieldFailure {
        field: field.to_string(),
        reason: reason.into(),
    })
}

fn run_cmd(cmd: &str, args: Value) -> Result<Value, PluginError> {
    if control::cancelled() {
        return Err(PluginError::Cancelled);
    }
    let raw = commands::run(cmd, &args.to_string())?;
    serde_json::from_str(&raw).map_err(|e| PluginError::Trap(format!("invalid JSON from {cmd}: {e}")))
}

fn validate_prefix(prefix: &str) -> Result<String, PluginError> {
    let trimmed = prefix.trim();
    if trimmed.is_empty() || trimmed.len() > 24 {
        return Err(invalid("prefix", "prefix must be 1 to 24 characters"));
    }
    let mut chars = trimmed.chars();
    let first = chars.next().ok_or_else(|| invalid("prefix", "prefix cannot be empty"))?;
    if !first.is_ascii_alphabetic() {
        return Err(invalid("prefix", "prefix must start with an ASCII letter"));
    }
    if !chars.all(|c| c.is_ascii_alphanumeric() || c == '_') {
        return Err(invalid(
            "prefix",
            "prefix may contain only ASCII letters, digits, and underscores",
        ));
    }
    Ok(trimmed.to_string())
}

fn list_existing_params() -> Result<BTreeSet<String>, PluginError> {
    let res = run_cmd("parameters.list", json!({}))?;
    let mut set = BTreeSet::new();
    if let Some(arr) = res.get("parameters").and_then(Value::as_array) {
        for item in arr {
            if let Some(name) = item.get("name").and_then(Value::as_str) {
                set.insert(name.to_string());
            }
        }
    }
    Ok(set)
}

fn list_existing_doc_names() -> Result<BTreeSet<String>, PluginError> {
    let res = run_cmd("document.inspect", json!({}))?;
    let mut set = BTreeSet::new();
    for key in ["sketches", "features", "bodies"] {
        if let Some(arr) = res.get(key).and_then(Value::as_array) {
            for item in arr {
                if let Some(name) = item.get("name").and_then(Value::as_str) {
                    set.insert(name.to_string());
                }
            }
        }
    }
    Ok(set)
}

fn choose_unique_prefix(
    requested: &str,
    existing_params: &BTreeSet<String>,
    existing_names: &BTreeSet<String>,
) -> String {
    let is_taken = |cand: &str| {
        existing_params.contains(&format!("{cand}_m"))
            || existing_names.contains(&format!("{cand}_ToothSketch"))
            || existing_names.contains(&format!("{cand}_PinionTooth"))
            || existing_names.contains(&format!("{cand}_Tooth"))
    };
    if !is_taken(requested) {
        return requested.to_string();
    }
    // If requested ends with digits (e.g. "g1" or "p1"), increment the number ("g2", "g3", ...).
    let base = requested.trim_end_matches(|c: char| c.is_ascii_digit());
    let num_part = &requested[base.len()..];
    if !base.is_empty() && !num_part.is_empty() {
        let start = num_part.parse::<u32>().unwrap_or(1) + 1;
        for n in start..1000 {
            let cand = format!("{base}{n}");
            if !is_taken(&cand) {
                return cand;
            }
        }
    }
    for n in 2..1000 {
        let cand = format!("{requested}_{n}");
        if !is_taken(&cand) {
            return cand;
        }
    }
    requested.to_string()
}

fn unique_name(desired: &str, existing_names: &mut BTreeSet<String>) -> String {
    if !existing_names.contains(desired) {
        existing_names.insert(desired.to_string());
        return desired.to_string();
    }
    for n in 2..1000 {
        let cand = format!("{desired}{n}");
        if !existing_names.contains(&cand) {
            existing_names.insert(cand.clone());
            return cand;
        }
    }
    desired.to_string()
}

fn upsert_param(
    existing: &mut BTreeSet<String>,
    name: &str,
    expression: &str,
    unit: &str,
    comment: &str,
    favorite: bool,
) -> Result<(), PluginError> {
    if existing.contains(name) {
        run_cmd(
            "parameters.change",
            json!({
                "name": name,
                "expression": expression,
                "unit": unit,
            }),
        )?;
    } else {
        run_cmd(
            "parameters.add",
            json!({
                "name": name,
                "expression": expression,
                "unit": unit,
                "comment": comment,
                "favorite": favorite,
            }),
        )?;
        existing.insert(name.to_string());
    }
    Ok(())
}

fn fmt_num(v: f64) -> String {
    if libm::fabs(v) < 1e-10 {
        return "0".into();
    }
    if libm::fabs(v - libm::round(v)) < 1e-9 {
        format!("{}", libm::round(v) as i64)
    } else {
        let s = format!("{v:.6}");
        s.trim_end_matches('0').trim_end_matches('.').to_string()
    }
}

fn v_add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn v_sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn v_scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn v_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn v_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn v_len(a: [f64; 3]) -> f64 {
    libm::sqrt(v_dot(a, a))
}

fn v_norm(a: [f64; 3]) -> Option<[f64; 3]> {
    let l = v_len(a);
    if l < 1e-9 {
        None
    } else {
        Some([a[0] / l, a[1] / l, a[2] / l])
    }
}

fn parse_vec3(v: &Value) -> Option<[f64; 3]> {
    let a = v.as_array()?;
    if a.len() < 3 {
        return None;
    }
    Some([a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?])
}

#[derive(Clone, Copy)]
struct PlacementFrame {
    origin: [f64; 3],
    x_dir: [f64; 3],
    y_dir: [f64; 3],
    z_dir: [f64; 3],
}

impl PlacementFrame {
    fn from_axis(origin: [f64; 3], axis: [f64; 3]) -> Self {
        let z = v_norm(axis).unwrap_or([0.0, 0.0, 1.0]);
        // Pick a stable reference vector not parallel to z.
        let ref_v = if libm::fabs(z[2]) < 0.9 {
            [0.0, 0.0, 1.0]
        } else {
            [1.0, 0.0, 0.0]
        };
        let x = v_norm(v_cross(ref_v, z)).unwrap_or([1.0, 0.0, 0.0]);
        let y = v_norm(v_cross(z, x)).unwrap_or([0.0, 1.0, 0.0]);
        Self { origin, x_dir: x, y_dir: y, z_dir: z }
    }

    fn from_base_plane(plane: &str) -> Self {
        match plane {
            "XZ" => Self {
                origin: [0.0, 0.0, 0.0],
                x_dir: [1.0, 0.0, 0.0],
                y_dir: [0.0, 0.0, -1.0],
                z_dir: [0.0, 1.0, 0.0],
            },
            "YZ" => Self {
                origin: [0.0, 0.0, 0.0],
                x_dir: [0.0, 1.0, 0.0],
                y_dir: [0.0, 0.0, 1.0],
                z_dir: [1.0, 0.0, 0.0],
            },
            _ => Self {
                origin: [0.0, 0.0, 0.0],
                x_dir: [1.0, 0.0, 0.0],
                y_dir: [0.0, 1.0, 0.0],
                z_dir: [0.0, 0.0, 1.0],
            },
        }
    }

    fn apply_adjustments(mut self, flip: bool, axial_offset: f64, layout_angle_deg: f64) -> Self {
        if flip {
            self.z_dir = v_scale(self.z_dir, -1.0);
            self.y_dir = v_scale(self.y_dir, -1.0);
        }
        if libm::fabs(axial_offset) > 1e-9 {
            self.origin = v_add(self.origin, v_scale(self.z_dir, axial_offset));
        }
        if libm::fabs(layout_angle_deg) > 1e-9 {
            let rad = layout_angle_deg * core::f64::consts::PI / 180.0;
            let c = libm::cos(rad);
            let s = libm::sin(rad);
            let nx = v_add(v_scale(self.x_dir, c), v_scale(self.y_dir, s));
            let ny = v_add(v_scale(self.x_dir, -s), v_scale(self.y_dir, c));
            self.x_dir = v_norm(nx).unwrap_or(self.x_dir);
            self.y_dir = v_norm(ny).unwrap_or(self.y_dir);
        }
        self
    }

    fn origin_json(&self, offset_param: Option<&str>) -> Value {
        self.origin_offset_json(offset_param, None)
    }

    fn origin_offset_json(&self, x_param: Option<&str>, z_param: Option<&str>) -> Value {
        if x_param.is_none() && z_param.is_none() {
            return json!([self.origin[0], self.origin[1], self.origin[2]]);
        }
        let comp = |o: f64, dx: f64, dz: f64| -> Value {
            let mut terms: Vec<String> = Vec::new();
            if libm::fabs(o) >= 1e-9 {
                terms.push(format!("{} mm", fmt_num(o)));
            }
            if let Some(xp) = x_param {
                if libm::fabs(dx) >= 1e-9 {
                    if libm::fabs(dx - 1.0) < 1e-9 {
                        terms.push(xp.to_string());
                    } else if libm::fabs(dx + 1.0) < 1e-9 {
                        terms.push(format!("-{xp}"));
                    } else {
                        terms.push(format!("{xp} * {}", fmt_num(dx)));
                    }
                }
            }
            if let Some(zp) = z_param {
                if libm::fabs(dz) >= 1e-9 {
                    if libm::fabs(dz - 1.0) < 1e-9 {
                        terms.push(zp.to_string());
                    } else if libm::fabs(dz + 1.0) < 1e-9 {
                        terms.push(format!("-{zp}"));
                    } else {
                        terms.push(format!("{zp} * {}", fmt_num(dz)));
                    }
                }
            }
            if terms.is_empty() {
                json!(0.0)
            } else {
                json!(terms.join(" + "))
            }
        };
        json!([
            comp(self.origin[0], self.x_dir[0], self.z_dir[0]),
            comp(self.origin[1], self.x_dir[1], self.z_dir[1]),
            comp(self.origin[2], self.x_dir[2], self.z_dir[2]),
        ])
    }
}

fn circumcenter_3d(a: [f64; 3], b: [f64; 3], c: [f64; 3]) -> Option<([f64; 3], [f64; 3])> {
    let ab = v_sub(b, a);
    let ac = v_sub(c, a);
    let n = v_cross(ab, ac);
    let n2 = v_dot(n, n);
    if n2 < 1e-10 {
        return None;
    }
    let ab2 = v_dot(ab, ab);
    let ac2 = v_dot(ac, ac);
    let term1 = v_scale(v_cross(n, ab), ac2);
    let term2 = v_scale(v_cross(ac, n), ab2);
    let offset = v_scale(v_add(term1, term2), 1.0 / (2.0 * n2));
    Some((v_add(a, offset), v_norm(n)?))
}

fn resolve_axis_by_name(name: &str) -> Result<PlacementFrame, PluginError> {
    let trimmed = name.trim();
    match trimmed {
        "X" | "x" => return Ok(PlacementFrame::from_axis([0.0, 0.0, 0.0], [1.0, 0.0, 0.0])),
        "Y" | "y" => return Ok(PlacementFrame::from_axis([0.0, 0.0, 0.0], [0.0, 1.0, 0.0])),
        "Z" | "z" => return Ok(PlacementFrame::from_axis([0.0, 0.0, 0.0], [0.0, 0.0, 1.0])),
        _ => {}
    }
    let res = run_cmd(
        "inspect.measure",
        json!({ "items": [{ "type": "axis", "name": trimmed }] }),
    )?;
    let item = res
        .get("items")
        .and_then(Value::as_array)
        .and_then(|a| a.first())
        .ok_or_else(|| invalid("axis_name", format!("axis `{trimmed}` not found")))?;
    let origin = item
        .get("origin")
        .and_then(parse_vec3)
        .ok_or_else(|| invalid("axis_name", format!("axis `{trimmed}` missing origin")))?;
    let dir = item
        .get("direction")
        .and_then(parse_vec3)
        .ok_or_else(|| invalid("axis_name", format!("axis `{trimmed}` missing direction")))?;
    Ok(PlacementFrame::from_axis(origin, dir))
}

fn resolve_placement(
    picks: &[Value],
    axis_name: &str,
    base_plane: &str,
    flip: bool,
    axial_offset: f64,
    layout_angle_deg: f64,
) -> Result<PlacementFrame, PluginError> {
    let mut frame = PlacementFrame::from_base_plane(base_plane);
    let active_pick = if let Some(p) = picks.first() {
        Some(p.clone())
    } else if axis_name.trim().is_empty() {
        // Auto-gather a pre-selected Axis, Face, Edge, or Sketch point from the viewport/Browser.
        run_cmd("selection.get", json!({ "detail": true }))
            .ok()
            .and_then(|r| r.get("items").and_then(Value::as_array).and_then(|a| a.first()).cloned())
    } else {
        None
    };
    if let Some(pick) = active_pick.as_ref() {
        let ptype = pick.get("type").and_then(Value::as_str).unwrap_or("");
        match ptype {
            "axis" => {
                if let (Some(o), Some(d)) = (
                    pick.get("origin").and_then(parse_vec3),
                    pick.get("direction").and_then(parse_vec3),
                ) {
                    frame = PlacementFrame::from_axis(o, d);
                } else if let Some(name) = pick.get("name").and_then(Value::as_str) {
                    frame = resolve_axis_by_name(name)?;
                }
            }
            "plane" => {
                if let Some(name) = pick.get("name").and_then(Value::as_str) {
                    match name {
                        "XY" | "XZ" | "YZ" => frame = PlacementFrame::from_base_plane(name),
                        other => {
                            if let Ok(res) = run_cmd(
                                "inspect.measure",
                                json!({ "items": [{ "type": "plane", "name": other }] }),
                            ) {
                                if let Some(item) = res.get("items").and_then(Value::as_array).and_then(|a| a.first()) {
                                    let o = item.get("origin").and_then(parse_vec3).unwrap_or([0.0, 0.0, 0.0]);
                                    let n = item.get("normal").and_then(parse_vec3).unwrap_or([0.0, 0.0, 1.0]);
                                    frame = PlacementFrame::from_axis(o, n);
                                }
                            }
                        }
                    }
                }
            }
            "face" => {
                let body = pick
                    .get("body")
                    .and_then(Value::as_str)
                    .ok_or_else(|| invalid("placement", "face pick missing body"))?;
                let faces_res = run_cmd("model.faces", json!({ "body": body }))?;
                let faces = faces_res
                    .get("faces")
                    .and_then(Value::as_array)
                    .ok_or_else(|| invalid("placement", "body has no faces"))?;
                let target_idx = pick.get("index").and_then(Value::as_u64);
                let face = target_idx
                    .and_then(|idx| faces.iter().find(|f| f.get("index").and_then(Value::as_u64) == Some(idx)))
                    .or_else(|| faces.first())
                    .ok_or_else(|| invalid("placement", "picked face not found"))?;
                if let Some(cyl) = face.get("cylinder").and_then(Value::as_object) {
                    let origin = cyl.get("origin").and_then(parse_vec3).unwrap_or([0.0, 0.0, 0.0]);
                    let axis = cyl.get("axis").and_then(parse_vec3).unwrap_or([0.0, 0.0, 1.0]);
                    frame = PlacementFrame::from_axis(origin, axis);
                } else if let Some(norm) = face.get("plane_normal").and_then(parse_vec3) {
                    let origin = face
                        .get("centroid")
                        .and_then(parse_vec3)
                        .or_else(|| pick.get("point").and_then(parse_vec3))
                        .unwrap_or([0.0, 0.0, 0.0]);
                    frame = PlacementFrame::from_axis(origin, norm);
                } else if let Some(pt) = pick.get("point").and_then(parse_vec3) {
                    frame.origin = pt;
                }
            }
            "edge" => {
                let body = pick
                    .get("body")
                    .and_then(Value::as_str)
                    .ok_or_else(|| invalid("placement", "edge pick missing body"))?;
                let edges_res = run_cmd("model.edges", json!({ "body": body }))?;
                let edges = edges_res
                    .get("edges")
                    .and_then(Value::as_array)
                    .ok_or_else(|| invalid("placement", "body has no edges"))?;
                let target_idx = pick.get("index").and_then(Value::as_u64);
                let edge = target_idx
                    .and_then(|idx| edges.iter().find(|e| e.get("index").and_then(Value::as_u64) == Some(idx)))
                    .or_else(|| edges.first())
                    .ok_or_else(|| invalid("placement", "picked edge not found"))?;
                let s = edge.get("start").and_then(parse_vec3).unwrap_or([0.0, 0.0, 0.0]);
                let m = edge.get("mid").and_then(parse_vec3).unwrap_or(s);
                let e = edge.get("end").and_then(parse_vec3).unwrap_or(s);
                if v_len(v_sub(s, e)) < 1e-4 && v_len(v_sub(s, m)) > 1e-4 {
                    // Full circular rim edge: center is midpoint of start & mid; find axis from adjacent face.
                    let center = v_scale(v_add(s, m), 0.5);
                    let mut axis = [0.0, 0.0, 1.0];
                    if let Ok(faces_res) = run_cmd("model.faces", json!({ "body": body })) {
                        if let Some(faces) = faces_res.get("faces").and_then(Value::as_array) {
                            let radial = v_norm(v_sub(s, center)).unwrap_or([1.0, 0.0, 0.0]);
                            for f in faces {
                                if let Some(n) = f.get("plane_normal").and_then(parse_vec3) {
                                    if let Some(fc) = f.get("centroid").and_then(parse_vec3) {
                                        if libm::fabs(v_dot(v_sub(center, fc), n)) < 1e-2
                                            && libm::fabs(v_dot(radial, n)) < 1e-2
                                        {
                                            axis = n;
                                            break;
                                        }
                                    }
                                } else if let Some(cyl) = f.get("cylinder").and_then(Value::as_object) {
                                    if let Some(ax) = cyl.get("axis").and_then(parse_vec3) {
                                        if libm::fabs(v_dot(radial, ax)) < 1e-2 {
                                            axis = ax;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                    }
                    frame = PlacementFrame::from_axis(center, axis);
                } else if let Some((center, norm)) = circumcenter_3d(s, m, e) {
                    frame = PlacementFrame::from_axis(center, norm);
                } else if let Some(dir) = v_norm(v_sub(e, s)) {
                    frame = PlacementFrame::from_axis(s, dir);
                }
            }
            "sketch_point" | "sketch_curve" => {
                if let Some(sk) = pick.get("sketch") {
                    if let Ok(sk_res) = run_cmd("sketch.inspect", json!({ "sketch": sk })) {
                        if let Some(plane) = sk_res.get("plane").and_then(Value::as_object) {
                            let o = plane.get("origin").and_then(parse_vec3).unwrap_or([0.0, 0.0, 0.0]);
                            let x = plane.get("x_dir").and_then(parse_vec3).and_then(v_norm).unwrap_or([1.0, 0.0, 0.0]);
                            let y = plane.get("y_dir").and_then(parse_vec3).and_then(v_norm).unwrap_or([0.0, 1.0, 0.0]);
                            let z = v_norm(v_cross(x, y)).unwrap_or([0.0, 0.0, 1.0]);
                            frame = PlacementFrame { origin: o, x_dir: x, y_dir: y, z_dir: z };
                        }
                    }
                }
                if let Some(pt) = pick.get("point").and_then(parse_vec3) {
                    frame.origin = pt;
                }
            }
            _ => {
                if let Some(pt) = pick.get("point").and_then(parse_vec3) {
                    frame.origin = pt;
                }
            }
        }
    } else if !axis_name.trim().is_empty() {
        frame = resolve_axis_by_name(axis_name)?;
    }
    Ok(frame.apply_adjustments(flip, axial_offset, layout_angle_deg))
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum ToothKind {
    Straight,
    Helical,
    Herringbone,
}

impl ToothKind {
    fn parse(s: &str) -> Self {
        match s {
            "helical" => Self::Helical,
            "herringbone" => Self::Herringbone,
            _ => Self::Straight,
        }
    }
}

struct GearSpec<'a> {
    derived_prefix: &'a str,
    m_param: &'a str,
    mt_param: &'a str,
    at_param: &'a str,
    z_param: &'a str,
    fw_param: &'a str,
    x_param: &'a str,
    bl_param: &'a str,
    bore_param: Option<&'a str>,
    phi_param: Option<&'a str>,
    tooth_kind: ToothKind,
    beta_param: Option<&'a str>,
    hand_sign: f64,
    frame: PlacementFrame,
    offset_x_param: Option<&'a str>,
    sketch_name: &'a str,
    path_sketch_name: &'a str,
    body_name: &'a str,
    m_val: f64,
    mt_val: f64,
    teeth_val: u32,
    shift_val: f64,
    bore_val: f64,
}

fn extract_curve_endpoints(res: &Value, label: &str) -> Result<(String, String), PluginError> {
    let pts = res
        .get("created")
        .and_then(|c| c.get("points"))
        .and_then(Value::as_array)
        .ok_or_else(|| PluginError::Trap(format!("{label}: missing created.points")))?;
    let first = pts
        .first()
        .and_then(Value::as_str)
        .ok_or_else(|| PluginError::Trap(format!("{label}: empty created.points")))?;
    let last = pts
        .last()
        .and_then(Value::as_str)
        .ok_or_else(|| PluginError::Trap(format!("{label}: missing end point")))?;
    Ok((first.to_string(), last.to_string()))
}

fn extract_created_curve_id(res: &Value, label: &str) -> Result<String, PluginError> {
    res.get("created")
        .and_then(|c| c.get("curves"))
        .and_then(Value::as_array)
        .and_then(|a| a.first())
        .and_then(Value::as_str)
        .map(ToString::to_string)
        .ok_or_else(|| PluginError::Trap(format!("{label}: missing created.curves[0]")))
}

fn pick_outer_profile_index(sketch_name: &str) -> Result<u64, PluginError> {
    let insp = run_cmd("sketch.inspect", json!({ "sketch": sketch_name }))?;
    let profiles = insp
        .get("profiles")
        .and_then(Value::as_array)
        .ok_or_else(|| PluginError::Trap(format!("{sketch_name}: no closed profiles found")))?;
    if profiles.is_empty() {
        return Err(PluginError::Trap(format!("{sketch_name}: gear profile did not close")));
    }
    // The gear body is the profile with the largest area (and holes when bore > 0).
    let best = profiles
        .iter()
        .max_by(|a, b| {
            let aa = a.get("area_mm2").and_then(Value::as_f64).unwrap_or(0.0);
            let ba = b.get("area_mm2").and_then(Value::as_f64).unwrap_or(0.0);
            aa.total_cmp(&ba)
        })
        .and_then(|p| p.get("index").and_then(Value::as_u64))
        .unwrap_or(0);
    Ok(best)
}

fn build_parametric_gear(
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

    // Validate basic geometry against numeric values before issuing CAD commands.
    let pitch_dia = spec.mt_val * spec.teeth_val as f64;
    let outer_dia = pitch_dia + 2.0 * spec.m_val * (1.0 + spec.shift_val);
    let root_dia = pitch_dia - 2.0 * spec.m_val * (1.25 - spec.shift_val);
    let at_rad_approx = 20.0 * core::f64::consts::PI / 180.0;
    let base_dia_approx = pitch_dia * libm::cos(at_rad_approx);
    if root_dia <= 0.5 {
        return Err(invalid("teeth", "root diameter is too small for this module/shift"));
    }
    if spec.bore_val > 0.0 && spec.bore_val >= root_dia - 0.5 {
        return Err(invalid(
            "bore_diameter",
            format!("bore ({:.2} mm) must be smaller than root diameter ({:.2} mm)", spec.bore_val, root_dia),
        ));
    }

    // Derived involute geometry parameters (auto-rebuilt by SolidCraft kernel).
    upsert_param(existing, &rp, &format!("{mt} * {z} / 2"), "mm", "Pitch radius", false)?;
    upsert_param(existing, &rb, &format!("{rp} * cos({at})"), "mm", "Base radius", false)?;
    upsert_param(existing, &ra, &format!("{rp} + {m} * (1 + {x})"), "mm", "Tip (addendum) radius", false)?;
    upsert_param(existing, &rf, &format!("{rp} - {m} * (1.25 - {x})"), "mm", "Root (dedendum) radius", false)?;
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
        "Involute tip roll parameter (rad)",
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
    upsert_param(
        existing,
        &psi,
        &format!("(pi / 2 + 2 * {x} * tan({at}) - {bl} / {mt}) / {z} + {inv_a}"),
        "",
        "Half tooth angular width on base circle (rad)",
        false,
    )?;
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

    // 1. Complete 2D gear cross-section sketch (1 tooth patterned in 2D via sketch.pattern.circular + bore).
    let plane_arg = json!({
        "origin": base_origin,
        "x_dir": [spec.frame.x_dir[0], spec.frame.x_dir[1], spec.frame.x_dir[2]],
        "y_dir": [spec.frame.y_dir[0], spec.frame.y_dir[1], spec.frame.y_dir[2]],
    });
    run_cmd(
        "sketch.create",
        json!({
            "plane": plane_arg,
            "name": spec.sketch_name,
        }),
    )?;

    // Draw bore circle BEFORE circular pattern so the sketch solver stays instant.
    if let Some(bore_p) = spec.bore_param {
        if spec.bore_val > 0.0 {
            run_cmd(
                "sketch.circle.center",
                json!({
                    "center": [0.0, 0.0],
                    "diameter": bore_p,
                }),
            )?;
        }
    }

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
        json!({
            "x": x_bot,
            "y": y_bot,
            "start": t_start,
            "end": t_end,
            "tolerance": 0.02,
        }),
    )?;
    let bot_curve = extract_created_curve_id(&bot_res, "bottom flank")?;
    let (bot_start, bot_end) = extract_curve_endpoints(&bot_res, "bottom flank")?;

    let top_res = run_cmd(
        "sketch.equation_curve",
        json!({
            "x": x_top,
            "y": y_top,
            "start": t_start,
            "end": t_end,
            "tolerance": 0.02,
        }),
    )?;
    let top_curve = extract_created_curve_id(&top_res, "top flank")?;
    let (top_start, top_end) = extract_curve_endpoints(&top_res, "top flank")?;

    // Tip arc connecting the two involute tips.
    let tip_res = run_cmd(
        "sketch.arc.three_point",
        json!({
            "start": bot_end,
            "through": tip_mid,
            "end": top_end,
            "connect": true,
        }),
    )?;
    let tip_curve = extract_created_curve_id(&tip_res, "tip arc")?;

    // Root circle points at +/- half-pitch (phi +/- pi/z) and root flank join.
    let ang = |expr: String| format!("({phi_prefix}{expr}) * 1 rad");
    let pt_on_rf = |a_expr: String| -> Value {
        json!([
            format!("{rf} * cos({a_expr})"),
            format!("{rf} * sin({a_expr})"),
        ])
    };
    let p_hp_neg = pt_on_rf(ang(format!("-{hp}")));
    let p_hp_pos = pt_on_rf(ang(hp.clone()));

    let mut pattern_entities: Vec<String> = Vec::new();
    pattern_entities.push(bot_curve);
    pattern_entities.push(top_curve);
    pattern_entities.push(tip_curve);

    let needs_radial = root_dia < base_dia_approx - 1e-3;
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
        pattern_entities.push(extract_created_curve_id(&a_bot, "bot root arc")?);
        pattern_entities.push(extract_created_curve_id(&l_bot, "bot root line")?);
        pattern_entities.push(extract_created_curve_id(&l_top, "top root line")?);
        pattern_entities.push(extract_created_curve_id(&a_top, "top root arc")?);
    } else {
        // Involute starts directly on root circle rf at angle phi -/+ (psi - inv(t_start)).
        let psi_f = format!("{gp}_psi_f");
        upsert_param(
            existing,
            &psi_f,
            &format!("{psi} - ({t_start} - atan({t_start}) / 1 rad)"),
            "",
            "Half tooth angular width on root circle (rad)",
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
        pattern_entities.push(extract_created_curve_id(&a_bot, "bot root arc")?);
        pattern_entities.push(extract_created_curve_id(&a_top, "top root arc")?);
    }

    // Parametric 2D circular pattern of all teeth around [0, 0] (ZERO 3D tooth booleans!).
    run_cmd(
        "sketch.pattern.circular",
        json!({
            "entities": pattern_entities,
            "center": [0.0, 0.0],
            "count": z,
            "angle": "360 deg",
        }),
    )?;
    run_cmd("sketch.finish", json!({}))?;

    let gear_profile = pick_outer_profile_index(spec.sketch_name)?;

    // 2. Build complete gear in ONE feature (single extrude for straight, single twisted sweep for helical/herringbone).
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

        // Axis path sketch on local XZ plane (x_dir = frame.x_dir, y_dir = frame.z_dir).
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
        let line_res = run_cmd(
            "sketch.line",
            json!({
                "points": [[0.0, 0.0], [0.0, &build_h]],
            }),
        )?;
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

    // 3. For herringbone, mirror bottom half across live midplane (z = fw / 2) and combine!
    if is_herringbone {
        let mid_origin = spec.frame.origin_offset_json(spec.offset_x_param, Some(&build_h));
        run_cmd(
            "solid.mirror",
            json!({
                "bodies": [&actual_body],
                "combine": true,
                "plane": {
                    "origin": mid_origin,
                    "normal": axis_vec,
                },
            }),
        )?;
    }

    Ok(json!({
        "body": actual_body,
        "sketch": spec.sketch_name,
        "pitch_diameter_mm": pitch_dia,
        "outer_diameter_mm": outer_dia,
        "root_diameter_mm": root_dia,
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
            profile_shift: 0.0,
            backlash: 0.05,
            body_name: "GM_Gear".into(),
        }
    }
}

fn setup_transverse_params(
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

fn run_cylindrical_gear(raw: &Value) -> Result<String, PluginError> {
    let inp: CylindricalGearInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    if inp.module <= 0.0 || inp.face_width <= 0.0 {
        return Err(invalid("module", "module and face_width must be positive"));
    }
    if inp.teeth < 8 || inp.teeth > 80 {
        return Err(invalid("teeth", "teeth must be between 8 and 80"));
    }
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

    upsert_param(&mut existing, &m_p, &fmt_num(inp.module), "mm", "Gear module", true)?;
    upsert_param(&mut existing, &z_p, &format!("{}", inp.teeth), "", "Tooth count", true)?;
    upsert_param(&mut existing, &a_p, &fmt_num(inp.pressure_angle), "deg", "Pressure angle", true)?;
    upsert_param(&mut existing, &fw_p, &fmt_num(inp.face_width), "mm", "Face width", true)?;
    upsert_param(&mut existing, &x_p, &fmt_num(inp.profile_shift), "", "Profile shift coefficient", false)?;
    upsert_param(&mut existing, &bl_p, &fmt_num(inp.backlash), "mm", "Backlash", false)?;
    if inp.bore_diameter > 0.0 {
        upsert_param(&mut existing, &bore_p, &fmt_num(inp.bore_diameter), "mm", "Bore diameter", true)?;
    }

    let (mt_p, at_p, beta_p, mt_val, _at_deg) = setup_transverse_params(
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
            bore_param: if inp.bore_diameter > 0.0 { Some(&bore_p) } else { None },
            phi_param: None,
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
            teeth_val: inp.teeth,
            shift_val: inp.profile_shift,
            bore_val: inp.bore_diameter,
        },
    )?;

    let out = json!({
        "mode": "gm-cylindrical-gear",
        "tooth_type": inp.tooth_type,
        "prefix": p,
        "gear": info,
        "editable_parameters": [m_p, z_p, a_p, fw_p, x_p, bl_p],
    });
    Ok(out.to_string())
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

fn run_parallel_pair(raw: &Value) -> Result<String, PluginError> {
    let inp: ParallelPairInput =
        serde_json::from_value(raw.clone()).map_err(|e| invalid("params", format!("invalid input: {e}")))?;
    let p = validate_prefix(&inp.prefix)?;
    if inp.module <= 0.0 || inp.face_width <= 0.0 {
        return Err(invalid("module", "module and face_width must be positive"));
    }
    if inp.teeth_1 < 8 || inp.teeth_1 > 80 || inp.teeth_2 < 8 || inp.teeth_2 > 80 {
        return Err(invalid("teeth_1", "tooth counts must be between 8 and 80"));
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

    // ISO 21771 center distance (nominal or backlash-shifted), gear ratio, and Gear 2 meshing phase.
    let cd_expr = if cd_backlash {
        format!("{mt_p} * ({z1_p} + {z2_p}) / 2 + {bl_p} / (2 * tan({at_p}))")
    } else {
        format!("{mt_p} * ({z1_p} + {z2_p}) / 2")
    };
    upsert_param(
        &mut existing,
        &cd_p,
        &cd_expr,
        "mm",
        "ISO 21771 shaft center distance",
        true,
    )?;
    upsert_param(
        &mut existing,
        &ratio_p,
        &format!("{z2_p} / {z1_p}"),
        "",
        "Gear ratio Z2 / Z1",
        true,
    )?;
    upsert_param(
        &mut existing,
        &phi2_p,
        &format!("pi + pi / {z2_p}"),
        "",
        "Gear 2 meshing phase angle (rad)",
        false,
    )?;

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
            phi_param: None,
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
            teeth_val: inp.teeth_1,
            shift_val: 0.0,
            bore_val: inp.bore_1,
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
            phi_param: Some(&phi2_p),
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
            teeth_val: inp.teeth_2,
            shift_val: 0.0,
            bore_val: inp.bore_2,
        },
    )?;

    let at_rad = at_deg * core::f64::consts::PI / 180.0;
    let cd_mm = mt_val * (inp.teeth_1 + inp.teeth_2) as f64 / 2.0
        + if cd_backlash { inp.backlash / (2.0 * libm::tan(at_rad)) } else { 0.0 };
    let ratio = inp.teeth_2 as f64 / inp.teeth_1 as f64;
    let out = json!({
        "mode": "gm-parallel-pair",
        "tooth_type": inp.tooth_type,
        "prefix": p,
        "center_distance_mm": cd_mm,
        "gear_ratio": ratio,
        "pinion": g1,
        "gear": g2,
        "editable_parameters": [m_p, z1_p, z2_p, a_p, fw_p, bl_p, cd_p, ratio_p],
    });
    Ok(out.to_string())
}

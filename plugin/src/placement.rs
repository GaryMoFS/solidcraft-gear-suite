use alloc::format;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use serde_json::{Value, json};

use crate::{PluginError, fmt_num, invalid, run_cmd};

pub fn v_add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub fn v_sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub fn v_scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

pub fn v_dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn v_cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

pub fn v_len(a: [f64; 3]) -> f64 {
    libm::sqrt(v_dot(a, a))
}

pub fn v_norm(a: [f64; 3]) -> Option<[f64; 3]> {
    let l = v_len(a);
    if l < 1e-9 {
        None
    } else {
        Some([a[0] / l, a[1] / l, a[2] / l])
    }
}

pub fn parse_vec3(v: &Value) -> Option<[f64; 3]> {
    let a = v.as_array()?;
    if a.len() < 3 {
        return None;
    }
    Some([a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?])
}

#[derive(Clone, Copy)]
pub struct PlacementFrame {
    pub origin: [f64; 3],
    pub x_dir: [f64; 3],
    pub y_dir: [f64; 3],
    pub z_dir: [f64; 3],
}

impl PlacementFrame {
    pub fn from_axis(origin: [f64; 3], axis: [f64; 3]) -> Self {
        let z = v_norm(axis).unwrap_or([0.0, 0.0, 1.0]);
        let ref_v = if libm::fabs(z[2]) < 0.9 {
            [0.0, 0.0, 1.0]
        } else {
            [1.0, 0.0, 0.0]
        };
        let x = v_norm(v_cross(ref_v, z)).unwrap_or([1.0, 0.0, 0.0]);
        let y = v_norm(v_cross(z, x)).unwrap_or([0.0, 1.0, 0.0]);
        Self { origin, x_dir: x, y_dir: y, z_dir: z }
    }

    pub fn from_base_plane(plane: &str) -> Self {
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

    pub fn apply_adjustments(mut self, flip: bool, axial_offset: f64, layout_angle_deg: f64) -> Self {
        if flip {
            self.z_dir = v_scale(self.z_dir, -1.0);
            self.y_dir = v_scale(self.y_dir, -1.0);
        }
        if libm::fabs(axial_offset) > 1e-9 {
            self.origin = v_add(self.origin, v_scale(self.z_dir, axial_offset));
        }
        if libm::fabs(layout_angle_deg) > 1e-9 {
            self = self.rotated_z(layout_angle_deg);
        }
        self
    }

    pub fn rotated_z(mut self, angle_deg: f64) -> Self {
        let rad = angle_deg * core::f64::consts::PI / 180.0;
        let c = libm::cos(rad);
        let s = libm::sin(rad);
        let nx = v_add(v_scale(self.x_dir, c), v_scale(self.y_dir, s));
        let ny = v_add(v_scale(self.x_dir, -s), v_scale(self.y_dir, c));
        self.x_dir = v_norm(nx).unwrap_or(self.x_dir);
        self.y_dir = v_norm(ny).unwrap_or(self.y_dir);
        self
    }

    /// Rotate frame about its local Y axis by `angle_rad` (useful for bevel gear shaft angle).
    pub fn rotated_y(mut self, angle_rad: f64) -> Self {
        let c = libm::cos(angle_rad);
        let s = libm::sin(angle_rad);
        let nx = v_add(v_scale(self.x_dir, c), v_scale(self.z_dir, -s));
        let nz = v_add(v_scale(self.x_dir, s), v_scale(self.z_dir, c));
        self.x_dir = v_norm(nx).unwrap_or(self.x_dir);
        self.z_dir = v_norm(nz).unwrap_or(self.z_dir);
        self
    }

    pub fn world_point(&self, lx: f64, ly: f64, lz: f64) -> [f64; 3] {
        v_add(
            v_add(self.origin, v_scale(self.x_dir, lx)),
            v_add(v_scale(self.y_dir, ly), v_scale(self.z_dir, lz)),
        )
    }

    pub fn origin_json(&self, offset_param: Option<&str>) -> Value {
        self.origin_offset_json(offset_param, None)
    }

    pub fn origin_offset_json(&self, x_param: Option<&str>, z_param: Option<&str>) -> Value {
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

    pub fn plane_json(&self, origin: Value) -> Value {
        json!({
            "origin": origin,
            "x_dir": [self.x_dir[0], self.x_dir[1], self.x_dir[2]],
            "y_dir": [self.y_dir[0], self.y_dir[1], self.y_dir[2]],
        })
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

pub fn resolve_axis_by_name(name: &str) -> Result<PlacementFrame, PluginError> {
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

pub fn resolve_placement(
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

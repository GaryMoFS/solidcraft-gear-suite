//! Assembly component & motion-link helpers (`component.from_bodies`, `joint.as_built`,
//! `joint.gear`, `joint.rack_pinion`, `joint.rigid_group`) for GM Gear Suite v0.8.0+.

use alloc::format;
use alloc::string::String;
use alloc::vec::Vec;
use serde_json::{Value, json};

use crate::placement::PlacementFrame;
use crate::{PluginError, run_cmd};

/// Convert a list of body names into individual assembly components via `component.from_bodies`.
pub fn make_components_from_bodies(bodies: &[String]) -> Result<(), PluginError> {
    if bodies.is_empty() {
        return Ok(());
    }
    run_cmd("component.from_bodies", json!({ "bodies": bodies }))?;
    Ok(())
}

/// Add an as-built revolute joint on `comp_name` relative to `"root"` at `(origin_3d, frame.z_dir)`.
pub fn add_revolute_joint(
    joint_name: &str,
    comp_name: &str,
    origin: [f64; 3],
    frame: &PlacementFrame,
) -> Result<(), PluginError> {
    run_cmd(
        "joint.as_built",
        json!({
            "type": "revolute",
            "a": comp_name,
            "b": "root",
            "name": joint_name,
            "at": {
                "frame": {
                    "origin": [origin[0], origin[1], origin[2]],
                    "z": [frame.z_dir[0], frame.z_dir[1], frame.z_dir[2]],
                    "x": [frame.x_dir[0], frame.x_dir[1], frame.x_dir[2]],
                }
            },
            "quiet": true,
        }),
    )?;
    Ok(())
}

/// Add an as-built slider joint on `comp_name` relative to `"root"` sliding along `slide_axis`.
pub fn add_slider_joint(
    joint_name: &str,
    comp_name: &str,
    origin: [f64; 3],
    slide_axis: [f64; 3],
    x_hint: [f64; 3],
) -> Result<(), PluginError> {
    run_cmd(
        "joint.as_built",
        json!({
            "type": "slider",
            "a": comp_name,
            "b": "root",
            "name": joint_name,
            "at": {
                "frame": {
                    "origin": [origin[0], origin[1], origin[2]],
                    "z": [slide_axis[0], slide_axis[1], slide_axis[2]],
                    "x": [x_hint[0], x_hint[1], x_hint[2]],
                }
            },
            "quiet": true,
        }),
    )?;
    Ok(())
}

/// Link two revolute joints with a parametric `joint.gear` motion link.
pub fn add_gear_link(
    joint_a: &str,
    joint_b: &str,
    ra_expr: &str,
    rb_expr: &str,
    internal: bool,
) -> Result<Value, PluginError> {
    run_cmd(
        "joint.gear",
        json!({
            "a": joint_a,
            "b": joint_b,
            "ra": ra_expr,
            "rb": rb_expr,
            "internal": internal,
            "quiet": true,
        }),
    )
}

/// Link a revolute pinion joint and a slider rack joint with `joint.rack_pinion`.
pub fn add_rack_pinion_link(
    pinion_joint: &str,
    rack_joint: &str,
    radius_expr: &str,
) -> Result<Value, PluginError> {
    run_cmd(
        "joint.rack_pinion",
        json!({
            "a": pinion_joint,
            "b": rack_joint,
            "radius": radius_expr,
            "quiet": true,
        }),
    )
}

/// Rigidly couple two coaxial components (e.g., compound intermediate shaft gear + next-stage pinion).
pub fn add_rigid_pair(comp_a: &str, comp_b: &str, name: &str) -> Result<(), PluginError> {
    run_cmd(
        "joint.as_built",
        json!({
            "type": "rigid",
            "a": comp_a,
            "b": comp_b,
            "name": name,
            "quiet": true,
        }),
    )?;
    Ok(())
}

/// Compute a 3D world point offset from `frame.origin` by `(dx, dy, dz)` in the frame's local basis.
pub fn offset_point(frame: &PlacementFrame, dx: f64, dy: f64, dz: f64) -> [f64; 3] {
    [
        frame.origin[0] + dx * frame.x_dir[0] + dy * frame.y_dir[0] + dz * frame.z_dir[0],
        frame.origin[1] + dx * frame.x_dir[1] + dy * frame.y_dir[1] + dz * frame.z_dir[1],
        frame.origin[2] + dx * frame.x_dir[2] + dy * frame.y_dir[2] + dz * frame.z_dir[2],
    ]
}

/// Build assembly components + revolute joints + `joint.gear` link for a 2-gear external pair.
pub fn setup_parallel_pair_joints(
    prefix: &str,
    body_1: &str,
    body_2: &str,
    z1_param: &str,
    z2_param: &str,
    cd_mm: f64,
    frame: &PlacementFrame,
) -> Result<Vec<String>, PluginError> {
    let bodies = alloc::vec![String::from(body_1), String::from(body_2)];
    make_components_from_bodies(&bodies)?;
    let j1 = format!("{prefix}_J1");
    let j2 = format!("{prefix}_J2");
    let p1 = offset_point(frame, 0.0, 0.0, 0.0);
    let p2 = offset_point(frame, cd_mm, 0.0, 0.0);
    add_revolute_joint(&j1, body_1, p1, frame)?;
    add_revolute_joint(&j2, body_2, p2, frame)?;
    add_gear_link(&j1, &j2, z1_param, z2_param, false)?;
    Ok(alloc::vec![j1, j2])
}

#![no_std]
#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

extern crate alloc;

mod bevel;
mod cycloidal;
mod cylindrical;
mod geneva;
mod joints;
mod knurl;
mod placement;
mod planetary;
mod rack;
pub mod panel;
pub mod solver;
mod train;
mod worm;

use alloc::collections::BTreeSet;
use alloc::format;
use alloc::string::{String, ToString};
use serde_json::{Value, json};

#[global_allocator]
static ALLOCATOR: dlmalloc::GlobalDlmalloc = dlmalloc::GlobalDlmalloc;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo<'_>) -> ! {
    core::arch::wasm32::unreachable()
}

wit_bindgen::generate!({
    path: "../wit/plugin.wit",
    world: "command-dialog-plugin",
});

use craft::plugin::commands;
use craft::plugin::control;
pub use craft::plugin::types::{FieldFailure, PluginError};
use exports::craft::plugin::dialog_provider::Guest as DialogGuest;
use exports::craft::plugin::entry::Guest as EntryGuest;

struct GearSuitePlugin;

export!(GearSuitePlugin);

impl EntryGuest for GearSuitePlugin {
    fn execute(entry: String, params: String) -> Result<String, PluginError> {
        let value: Value =
            serde_json::from_str(&params).map_err(|e| invalid("params", format!("invalid JSON: {e}")))?;
        match entry.as_str() {
            "gm-cylindrical-gear" | "parametric-spur-gear" => cylindrical::run_cylindrical_gear(&value),
            "gm-parallel-pair" | "parametric-parallel-pair" => cylindrical::run_parallel_pair(&value),
            "gm-planetary" => planetary::run_planetary(&value),
            "gm-rack-pinion" => rack::run_rack_pinion(&value),
            "gm-cycloidal" => cycloidal::run_cycloidal(&value),
            "gm-bevel-pair" => bevel::run_bevel_pair(&value),
            "gm-worm-drive" => worm::run_worm_drive(&value),
            "gm-knurl" => knurl::run_knurl(&value),
            "gm-geneva" => geneva::run_geneva(&value),
            "gm-gear-train" => train::run_gear_train(&value),
            _ => Err(invalid("entry", format!("unknown entry `{entry}`"))),
        }
    }
}

impl DialogGuest for GearSuitePlugin {
    fn update_fields(entry: String, values: String) -> Result<String, PluginError> {
        let value: Value = serde_json::from_str(&values).unwrap_or_else(|_| json!({}));
        let patches = panel::compute_dialog_hints(&entry, &value);
        Ok(patches.to_string())
    }
}

pub fn report_progress(fraction: f32, msg: &str) -> Result<(), PluginError> {
    if control::cancelled() {
        return Err(PluginError::Cancelled);
    }
    control::progress(fraction.clamp(0.0, 1.0), msg);
    Ok(())
}

pub fn is_preview(raw: &Value) -> bool {
    raw.get("fast_preview")
        .or_else(|| raw.get("_preview"))
        .and_then(Value::as_bool)
        .unwrap_or(false)
}

pub fn invalid(field: &str, reason: impl Into<String>) -> PluginError {
    PluginError::InvalidInput(FieldFailure {
        field: field.to_string(),
        reason: reason.into(),
    })
}

pub fn run_cmd(cmd: &str, args: Value) -> Result<Value, PluginError> {
    if control::cancelled() {
        return Err(PluginError::Cancelled);
    }
    let raw = commands::run(cmd, &args.to_string())?;
    serde_json::from_str(&raw).map_err(|e| PluginError::Trap(format!("invalid JSON from {cmd}: {e}")))
}

pub fn validate_prefix(prefix: &str) -> Result<String, PluginError> {
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

pub fn list_existing_params() -> Result<BTreeSet<String>, PluginError> {
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

pub fn list_existing_doc_names() -> Result<BTreeSet<String>, PluginError> {
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

pub fn choose_unique_prefix(
    requested: &str,
    existing_params: &BTreeSet<String>,
    existing_names: &BTreeSet<String>,
) -> String {
    let is_taken = |cand: &str| {
        existing_params.contains(&format!("{cand}_m"))
            || existing_params.contains(&format!("{cand}_R"))
            || existing_names.contains(&format!("{cand}_ToothSketch"))
            || existing_names.contains(&format!("{cand}_PinionTooth"))
            || existing_names.contains(&format!("{cand}_SunSketch"))
            || existing_names.contains(&format!("{cand}_DiscSketch"))
    };
    if !is_taken(requested) {
        return requested.to_string();
    }
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

pub fn unique_name(desired: &str, existing_names: &mut BTreeSet<String>) -> String {
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

pub fn upsert_param(
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
            json!({ "name": name, "expression": expression, "unit": unit }),
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

pub fn fmt_num(v: f64) -> String {
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

pub fn extract_curve_endpoints(res: &Value, label: &str) -> Result<(String, String), PluginError> {
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

pub fn extract_created_curve_id(res: &Value, label: &str) -> Result<String, PluginError> {
    res.get("created")
        .and_then(|c| c.get("curves"))
        .and_then(Value::as_array)
        .and_then(|a| a.first())
        .and_then(Value::as_str)
        .map(ToString::to_string)
        .ok_or_else(|| PluginError::Trap(format!("{label}: missing created.curves[0]")))
}

pub fn pick_outer_profile_index(sketch_name: &str, prefer_with_holes: bool) -> Result<u64, PluginError> {
    let insp = run_cmd("sketch.inspect", json!({ "sketch": sketch_name }))?;
    let profiles = insp
        .get("profiles")
        .and_then(Value::as_array)
        .ok_or_else(|| PluginError::Trap(format!("{sketch_name}: no closed profiles found")))?;
    if profiles.is_empty() {
        return Err(PluginError::Trap(format!("{sketch_name}: gear profile did not close")));
    }
    if prefer_with_holes {
        if let Some(ring) = profiles.iter().find(|p| {
            p.get("holes")
                .and_then(Value::as_array)
                .is_some_and(|h| !h.is_empty())
        }) {
            if let Some(idx) = ring.get("index").and_then(Value::as_u64) {
                return Ok(idx);
            }
        }
    }
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

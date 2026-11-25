//! Docked tabbed `GM Gear Engineering` panel (`ui@0.1`) and live dialog hints (`dialog@0.1`).

use alloc::format;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use serde_json::{Map, Value, json};

use crate::craft::plugin::types::{Datum, Value as UiValue};
use crate::craft::plugin::ui::{
    self, Column, Declarations, DockZone, Layout, Node, PanelSpec, SelectionMode, Tab, TableSpec,
    Widget,
};
use crate::solver;

pub const PANEL_ID: &str = "gm-engineering";

/// Static panel declaration — must match `runtime.ui` in `plugin.json` verbatim.
pub fn describe_panel() -> Declarations {
    Declarations {
        ui_version: 1,
        panels: vec![PanelSpec {
            id: PANEL_ID.into(),
            title: "GM Gear Engineering".into(),
            zones: vec![DockZone::Right, DockZone::Bottom],
            layout: Layout {
                root: 0,
                nodes: vec![
                    Node {
                        widget: Widget::Tabs(vec![
                            Tab {
                                label: "Geometry".into(),
                                child: 1,
                            },
                            Tab {
                                label: "Strength & Safety".into(),
                                child: 2,
                            },
                            Tab {
                                label: "Warnings".into(),
                                child: 3,
                            },
                        ]),
                    },
                    Node {
                        widget: Widget::Table(TableSpec {
                            bind: "/geometry".into(),
                            row_id: "id".into(),
                            columns: vec![
                                Column {
                                    key: "metric".into(),
                                    label: "Parameter".into(),
                                    unit: None,
                                },
                                Column {
                                    key: "value".into(),
                                    label: "Value".into(),
                                    unit: None,
                                },
                            ],
                            select: SelectionMode::None,
                            on_select: None,
                            on_edit: None,
                        }),
                    },
                    Node {
                        widget: Widget::Table(TableSpec {
                            bind: "/strength".into(),
                            row_id: "id".into(),
                            columns: vec![
                                Column {
                                    key: "metric".into(),
                                    label: "Check".into(),
                                    unit: None,
                                },
                                Column {
                                    key: "value".into(),
                                    label: "Value".into(),
                                    unit: None,
                                },
                            ],
                            select: SelectionMode::None,
                            on_select: None,
                            on_edit: None,
                        }),
                    },
                    Node {
                        widget: Widget::Table(TableSpec {
                            bind: "/warnings".into(),
                            row_id: "id".into(),
                            columns: vec![
                                Column {
                                    key: "level".into(),
                                    label: "Status".into(),
                                    unit: None,
                                },
                                Column {
                                    key: "message".into(),
                                    label: "Engineering Note".into(),
                                    unit: None,
                                },
                            ],
                            select: SelectionMode::None,
                            on_select: None,
                            on_edit: None,
                        }),
                    },
                ],
            },
        }],
        workspaces: vec![],
    }
}

/// Push engineering rows into the docked `GM Gear Engineering` panel.
/// Silently no-ops on headless hosts where panel UI is unavailable.
pub fn publish_panel(
    geometry_rows: &[(String, String)],
    strength_rows: &[(String, String)],
    warning_rows: &[(String, String)],
) {
    let mut datums: Vec<Datum> = Vec::new();
    for (i, (metric, val)) in geometry_rows.iter().enumerate() {
        datums.push(Datum {
            path: format!("/geometry/{i}/id"),
            value: UiValue::Text(format!("g{i}")),
        });
        datums.push(Datum {
            path: format!("/geometry/{i}/metric"),
            value: UiValue::Text(metric.clone()),
        });
        datums.push(Datum {
            path: format!("/geometry/{i}/value"),
            value: UiValue::Text(val.clone()),
        });
    }
    for (i, (metric, val)) in strength_rows.iter().enumerate() {
        datums.push(Datum {
            path: format!("/strength/{i}/id"),
            value: UiValue::Text(format!("s{i}")),
        });
        datums.push(Datum {
            path: format!("/strength/{i}/metric"),
            value: UiValue::Text(metric.clone()),
        });
        datums.push(Datum {
            path: format!("/strength/{i}/value"),
            value: UiValue::Text(val.clone()),
        });
    }
    if warning_rows.is_empty() {
        datums.push(Datum {
            path: "/warnings/0/id".into(),
            value: UiValue::Text("w0".into()),
        });
        datums.push(Datum {
            path: "/warnings/0/level".into(),
            value: UiValue::Text("OK".into()),
        });
        datums.push(Datum {
            path: "/warnings/0/message".into(),
            value: UiValue::Text("No engineering warnings.".into()),
        });
    } else {
        for (i, (lvl, msg)) in warning_rows.iter().enumerate() {
            datums.push(Datum {
                path: format!("/warnings/{i}/id"),
                value: UiValue::Text(format!("w{i}")),
            });
            datums.push(Datum {
                path: format!("/warnings/{i}/level"),
                value: UiValue::Text(lvl.clone()),
            });
            datums.push(Datum {
                path: format!("/warnings/{i}/message"),
                value: UiValue::Text(msg.clone()),
            });
        }
    }

    let _ = ui::clear_data(PANEL_ID);
    if ui::set_data(PANEL_ID, &datums).is_ok() {
        let _ = ui::show_panel(PANEL_ID, true);
    }
}

/// Publish a `MeshSolution` for a parallel gear pair to the docked panel.
pub fn publish_pair_report(
    label: &str,
    m_n: f64,
    m_t: f64,
    z1: u32,
    z2: u32,
    r: &solver::MeshSolution,
) {
    let ratio = z2 as f64 / z1 as f64;
    let mut geo = vec![
        ("Gear Set".into(), label.into()),
        (
            "Module (normal / transv.)".into(),
            format!("{m_n:.3} mm / {m_t:.3} mm"),
        ),
        (
            "Teeth (z1 / z2) & Ratio".into(),
            format!("{z1} / {z2}  (u = {ratio:.3}:1)"),
        ),
        (
            "Pitch Dia (d1 / d2)".into(),
            format!("{:.3} mm / {:.3} mm", m_t * z1 as f64, m_t * z2 as f64),
        ),
        (
            "Center Distance (std -> op)".into(),
            format!(
                "{:.3} mm -> {:.4} mm",
                r.standard_center_distance_mm, r.operating_center_distance_mm
            ),
        ),
        (
            "Operating Pressure Angle".into(),
            format!("{:.3} deg", r.operating_pressure_angle_deg),
        ),
        (
            "Contact Ratio (eps_a / eps_b / tot)".into(),
            format!(
                "{:.3} / {:.3} / {:.3}",
                r.contact_ratio_transverse, r.contact_ratio_overlap, r.contact_ratio_total
            ),
        ),
        (
            "Face Width Ratio (b / mn)".into(),
            format!("{:.2}", r.face_width_ratio),
        ),
    ];
    if r.tip_shortening_mm > 1e-4 {
        geo.push((
            "Tip Shortening".into(),
            format!("{:.4} mm", r.tip_shortening_mm),
        ));
    }

    let strength = if let Some(s) = &r.strength {
        let mat = s.get("material").and_then(Value::as_str).unwrap_or("—");
        let ft = s.get("tangential_load_n").and_then(Value::as_f64).unwrap_or(0.0);
        let vmps = s.get("pitch_line_velocity_mps").and_then(Value::as_f64).unwrap_or(0.0);
        let sb = s.get("lewis_bending_mpa").and_then(Value::as_f64).unwrap_or(0.0);
        let sfb = s.get("bending_safety_factor").and_then(Value::as_f64).unwrap_or(0.0);
        let sh = s.get("hertz_contact_mpa").and_then(Value::as_f64).unwrap_or(0.0);
        let sfh = s.get("contact_safety_factor").and_then(Value::as_f64).unwrap_or(0.0);
        vec![
            ("Material".into(), mat.into()),
            ("Tangential Force (Ft)".into(), format!("{ft:.1} N")),
            ("Pitch Line Velocity".into(), format!("{vmps:.2} m/s")),
            (
                "Lewis Bending Stress".into(),
                format!("{sb:.1} MPa  (SF = {sfb:.2})"),
            ),
            (
                "Hertzian Contact Stress".into(),
                format!("{sh:.1} MPa  (SH = {sfh:.2})"),
            ),
            (
                "Preliminary Verdict".into(),
                if sfb >= 1.5 && sfh >= 1.2 {
                    "PASS (preliminary)".into()
                } else if sfb >= 1.0 && sfh >= 1.0 {
                    "MARGINAL (review load / face width)".into()
                } else {
                    "UNDER-RATED (increase m, b, or material)".into()
                },
            ),
        ]
    } else {
        vec![(
            "Strength Check".into(),
            "Set Input Torque > 0 N·m to run Lewis/Hertz check.".into(),
        )]
    };

    let warnings: Vec<(String, String)> = r
        .warnings
        .iter()
        .map(|w| ("WARN".into(), w.clone()))
        .collect();

    publish_panel(&geo, &strength, &warnings);
}

/// Publish single cylindrical gear geometry & warnings to the docked panel.
pub fn publish_single_gear_report(
    label: &str,
    m_n: f64,
    m_t: f64,
    z: u32,
    alpha_deg: f64,
    r: &solver::MeshSolution,
) {
    let d = m_t * z as f64;
    let geo = vec![
        ("Gear".into(), label.into()),
        (
            "Module (normal / transv.)".into(),
            format!("{m_n:.3} mm / {m_t:.3} mm"),
        ),
        ("Teeth (z)".into(), format!("{z}")),
        ("Pitch Diameter (d)".into(), format!("{d:.3} mm")),
        ("Pressure Angle".into(), format!("{alpha_deg:.2} deg")),
        (
            "Undercut Limit (z_min)".into(),
            format!("{:.1} (x_min = {:.2})", r.z1_min_no_undercut, r.x1_min_recommended),
        ),
        (
            "Face Width Ratio (b / mn)".into(),
            format!("{:.2}", r.face_width_ratio),
        ),
    ];
    let strength = vec![(
        "Strength Check".into(),
        "Use GM Parallel Pair with Input Torque > 0 N·m for Lewis/Hertz check.".into(),
    )];
    let warnings: Vec<(String, String)> = r
        .warnings
        .iter()
        .map(|w| ("WARN".into(), w.clone()))
        .collect();
    publish_panel(&geo, &strength, &warnings);
}

/// Live `dialog@0.1` hook: computes real-time engineering hints as the user edits dialog fields.
pub fn compute_dialog_hints(entry: &str, values: &Value) -> Value {
    let mut patches = Map::new();
    match entry {
        "gm-cylindrical-gear" => {
            let sizing_unit = values
                .get("sizing_unit")
                .and_then(Value::as_str)
                .unwrap_or("module");
            let m_in = values.get("module").and_then(Value::as_f64).unwrap_or(2.0);
            let dp = values
                .get("diametral_pitch")
                .and_then(Value::as_f64)
                .unwrap_or(12.7);
            let cp = values
                .get("circular_pitch")
                .and_then(Value::as_f64)
                .unwrap_or(6.283);
            let Ok(m_n) = solver::resolve_module_mm(sizing_unit, m_in, dp, cp) else {
                return Value::Object(patches);
            };
            let z = values.get("teeth").and_then(Value::as_u64).unwrap_or(20) as u32;
            let alpha = values
                .get("pressure_angle")
                .and_then(Value::as_f64)
                .unwrap_or(20.0);
            let fw = values
                .get("face_width")
                .and_then(Value::as_f64)
                .unwrap_or(10.0);
            let x = values
                .get("profile_shift")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            let tooth_type = values
                .get("tooth_type")
                .and_then(Value::as_str)
                .unwrap_or("straight");
            let beta = if tooth_type == "straight" {
                0.0
            } else {
                values
                    .get("helix_angle")
                    .and_then(Value::as_f64)
                    .unwrap_or(15.0)
            };
            let mod_sys = values
                .get("module_system")
                .and_then(Value::as_str)
                .unwrap_or("normal");
            let gear_side = values
                .get("gear_side")
                .and_then(Value::as_str)
                .unwrap_or("external");

            let cos_b = libm::cos(beta.to_radians());
            let m_t = if tooth_type == "straight" || mod_sys == "transverse" {
                m_n
            } else {
                m_n / cos_b
            };
            let d = m_t * (z as f64);
            let da = if gear_side == "internal" {
                d - 2.0 * m_n * (1.0 - x)
            } else {
                d + 2.0 * m_n * (1.0 + x)
            };
            let df = if gear_side == "internal" {
                d + 2.0 * m_n * (1.25 + x)
            } else {
                d - 2.0 * m_n * (1.25 - x)
            };

            if sizing_unit == "dp" {
                patches.insert(
                    "diametral_pitch".into(),
                    json!({ "hint": format!("Equivalent module m = {m_n:.3} mm") }),
                );
            } else if sizing_unit == "cp" {
                patches.insert(
                    "circular_pitch".into(),
                    json!({ "hint": format!("Equivalent module m = {m_n:.3} mm") }),
                );
            } else {
                let nearest = solver::nearest_iso54_module(m_n);
                if libm::fabs(m_n - nearest) > 1e-3 {
                    patches.insert(
                        "module".into(),
                        json!({ "hint": format!("Non-ISO 54 Series 1 (nearest standard: {nearest} mm)") }),
                    );
                }
            }

            patches.insert(
                "teeth".into(),
                json!({ "hint": format!("Pitch d = {d:.2} mm | Tip da = {da:.2} mm | Root df = {df:.2} mm") }),
            );

            if gear_side == "external" {
                let sin_a = libm::sin(alpha.to_radians());
                let z_min = (2.0 / (sin_a * sin_a)) * cos_b * cos_b * cos_b;
                let x_min = (z_min - z as f64) / z_min;
                if (z as f64) < z_min && x < x_min - 0.02 {
                    patches.insert(
                        "profile_shift".into(),
                        json!({ "hint": format!("Undercut risk (z < {z_min:.1}): recommend x >= +{x_min:.2}") }),
                    );
                }
            }

            let bm = fw / m_n;
            if bm < 5.0 || bm > 16.0 {
                patches.insert(
                    "face_width".into(),
                    json!({ "hint": format!("b/m = {bm:.1} (typical range: 6..15)") }),
                );
            }
        }
        "gm-parallel-pair" => {
            let sizing_unit = values
                .get("sizing_unit")
                .and_then(Value::as_str)
                .unwrap_or("module");
            let m_in = values.get("module").and_then(Value::as_f64).unwrap_or(2.0);
            let dp = values
                .get("diametral_pitch")
                .and_then(Value::as_f64)
                .unwrap_or(12.7);
            let cp = values
                .get("circular_pitch")
                .and_then(Value::as_f64)
                .unwrap_or(6.283);
            let Ok(m_n) = solver::resolve_module_mm(sizing_unit, m_in, dp, cp) else {
                return Value::Object(patches);
            };
            let z1 = values.get("teeth_1").and_then(Value::as_u64).unwrap_or(16) as u32;
            let sizing_mode = values
                .get("sizing_mode")
                .and_then(Value::as_str)
                .unwrap_or("by_teeth");
            let z2 = if sizing_mode == "by_ratio" {
                let r = values
                    .get("gear_ratio")
                    .and_then(Value::as_f64)
                    .unwrap_or(2.0);
                libm::round((z1 as f64) * r).max(8.0) as u32
            } else {
                values.get("teeth_2").and_then(Value::as_u64).unwrap_or(32) as u32
            };
            let alpha = values
                .get("pressure_angle")
                .and_then(Value::as_f64)
                .unwrap_or(20.0);
            let fw = values
                .get("face_width")
                .and_then(Value::as_f64)
                .unwrap_or(10.0);
            let x1 = values
                .get("profile_shift_1")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            let x2 = values
                .get("profile_shift_2")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            let tooth_type = values
                .get("tooth_type")
                .and_then(Value::as_str)
                .unwrap_or("straight");
            let beta = if tooth_type == "straight" {
                0.0
            } else {
                values
                    .get("helix_angle")
                    .and_then(Value::as_f64)
                    .unwrap_or(15.0)
            };
            let mod_sys = values
                .get("module_system")
                .and_then(Value::as_str)
                .unwrap_or("normal");
            let mat = values
                .get("material")
                .and_then(Value::as_str)
                .unwrap_or("steel_1045");
            let torque = values
                .get("input_torque_nm")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);
            let rpm = values
                .get("input_rpm")
                .and_then(Value::as_f64)
                .unwrap_or(0.0);

            let cos_b = libm::cos(beta.to_radians());
            let (m_t, alpha_t_deg) = if tooth_type == "straight" || mod_sys == "transverse" {
                (m_n, alpha)
            } else {
                let at = libm::atan(libm::tan(alpha.to_radians()) / cos_b).to_degrees();
                (m_n / cos_b, at)
            };
            let r = solver::solve_external_pair(
                m_n,
                m_t,
                z1,
                z2,
                alpha,
                alpha_t_deg,
                beta,
                x1,
                x2,
                fw,
                0.0,
                mat,
                torque,
                rpm,
            );
            {
                if sizing_unit == "dp" {
                    patches.insert(
                        "diametral_pitch".into(),
                        json!({ "hint": format!("Equivalent module m = {m_n:.3} mm") }),
                    );
                } else if sizing_unit == "cp" {
                    patches.insert(
                        "circular_pitch".into(),
                        json!({ "hint": format!("Equivalent module m = {m_n:.3} mm") }),
                    );
                }

                let ratio = z2 as f64 / z1 as f64;
                let cd_hint = format!(
                    "CD aw = {:.3} mm (alpha_w = {:.2} deg) | eps_a = {:.2} (tot {:.2}) | u = {:.3}:1",
                    r.operating_center_distance_mm,
                    r.operating_pressure_angle_deg,
                    r.contact_ratio_transverse,
                    r.contact_ratio_total,
                    ratio
                );
                if sizing_mode == "by_ratio" {
                    patches.insert("gear_ratio".into(), json!({ "hint": cd_hint }));
                } else {
                    patches.insert("teeth_2".into(), json!({ "hint": cd_hint }));
                }

                if x1 < r.x1_min_recommended - 0.01 {
                    patches.insert(
                        "profile_shift_1".into(),
                        json!({ "hint": format!("Pinion undercut risk (z1 < {:.1}): suggest x1 >= +{:.2}", r.z1_min_no_undercut, r.x1_min_recommended) }),
                    );
                }

                if let Some(s) = &r.strength {
                    let sfb = s.get("bending_safety_factor").and_then(Value::as_f64).unwrap_or(0.0);
                    let sb = s.get("lewis_bending_mpa").and_then(Value::as_f64).unwrap_or(0.0);
                    let sfh = s.get("contact_safety_factor").and_then(Value::as_f64).unwrap_or(0.0);
                    let sh = s.get("hertz_contact_mpa").and_then(Value::as_f64).unwrap_or(0.0);
                    let verdict = if sfb >= 1.5 && sfh >= 1.2 {
                        "PASS"
                    } else if sfb >= 1.0 && sfh >= 1.0 {
                        "MARGINAL"
                    } else {
                        "UNDER-RATED"
                    };
                    patches.insert(
                        "input_torque_nm".into(),
                        json!({ "hint": format!("{verdict}: Bending SF={sfb:.2} ({sb:.0} MPa), Contact SH={sfh:.2} ({sh:.0} MPa)") }),
                    );
                }
            }
        }
        _ => {}
    }
    Value::Object(patches)
}

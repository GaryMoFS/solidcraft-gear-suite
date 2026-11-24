use std::fs::{self, File};
use std::io::Write;
use std::path::PathBuf;

use anyhow::{Context, Result};
use wit_component::{ComponentEncoder, StringEncoding};
use wit_parser::Resolve;
use zip::write::SimpleFileOptions;
use zip::ZipWriter;

fn main() -> Result<()> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .context("missing parent")?
        .to_path_buf();
    let core_wasm =
        root.join("plugin/target/wasm32-unknown-unknown/release/solidcraft_gear_suite_plugin.wasm");
    let wit_path = root.join("wit/plugin.wit");
    let manifest_path = root.join("plugin.json");

    let mut wasm_bytes =
        fs::read(&core_wasm).with_context(|| format!("read {}", core_wasm.display()))?;
    let mut resolve = Resolve::default();
    let (pkg_id, _) = resolve
        .push_path(&wit_path)
        .with_context(|| format!("parse {}", wit_path.display()))?;
    let world_id = resolve.select_world(&[pkg_id], Some("command-plugin"))?;

    wit_component::embed_component_metadata(&mut wasm_bytes, &resolve, world_id, StringEncoding::UTF8)?;
    let component = ComponentEncoder::default()
        .module(&wasm_bytes)?
        .validate(true)
        .encode()?;

    let dist_dir = root.join("dist/solidcraft-gear-suite");
    fs::create_dir_all(&dist_dir)?;
    let out_wasm = dist_dir.join("solidcraft-gear-suite.wasm");
    let out_manifest = dist_dir.join("plugin.json");
    let manifest_bytes = fs::read(&manifest_path)?;

    fs::write(&out_wasm, &component)?;
    fs::write(&out_manifest, &manifest_bytes)?;

    let zip_path = root.join("dist/solidcraft-gear-suite.zip");
    let file = File::create(&zip_path)?;
    let mut zip = ZipWriter::new(file);
    let opts = SimpleFileOptions::default().compression_method(zip::CompressionMethod::Deflated);

    zip.start_file("solidcraft-gear-suite/plugin.json", opts)?;
    zip.write_all(&manifest_bytes)?;
    zip.start_file("solidcraft-gear-suite/solidcraft-gear-suite.wasm", opts)?;
    zip.write_all(&component)?;
    zip.finish()?;

    println!("Built component: {} ({} bytes)", out_wasm.display(), component.len());
    println!("Packaged zip:    {}", zip_path.display());
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;
    use serde_json::{Value, json};
    use solidcraft_engine::Session;

    fn dist_and_approval(session: &mut Session) -> (PathBuf, String) {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).parent().unwrap().to_path_buf();
        let pkg_dir = root.join("dist/solidcraft-gear-suite");
        let zip_path = root.join("dist/solidcraft-gear-suite.zip");
        assert!(pkg_dir.join("plugin.json").exists(), "run packager first");
        assert!(zip_path.exists(), "run packager first");

        let inspect = session
            .execute("plugin.inspect", &json!({ "path": pkg_dir.to_string_lossy() }))
            .expect("plugin.inspect should succeed");
        assert_eq!(inspect["within_host_limits"], json!(true));
        let approval = inspect["approval"].as_str().expect("approval hash").to_string();
        (pkg_dir, approval)
    }

    fn body_bbox(session: &mut Session, body_name: &str) -> ([f64; 3], [f64; 3]) {
        let res = session.execute("document.inspect", &json!({})).expect("document.inspect");
        let bodies = res["bodies"].as_array().expect("bodies array");
        let body = bodies
            .iter()
            .find(|b| b["name"].as_str() == Some(body_name))
            .unwrap_or_else(|| panic!("body `{body_name}` not found in {bodies:?}"));
        let bb = &body["bbox"];
        let min = [
            bb["min"][0].as_f64().unwrap(),
            bb["min"][1].as_f64().unwrap(),
            bb["min"][2].as_f64().unwrap(),
        ];
        let max = [
            bb["max"][0].as_f64().unwrap(),
            bb["max"][1].as_f64().unwrap(),
            bb["max"][2].as_f64().unwrap(),
        ];
        (min, max)
    }

    #[test]
    fn parametric_spur_gear_and_live_kernel_auto_rebuild() {
        let mut session = Session::default();
        let (pkg_dir, approval) = dist_and_approval(&mut session);

        // 1. Run parametric-spur-gear: m=2, z=16, fw=10, bore=6 -> outer diameter = 2*(16+2) = 36 mm
        let res: Value = session
            .execute(
                "plugin.run",
                &json!({
                    "path": pkg_dir.to_string_lossy(),
                    "approval": approval,
                    "entry": "parametric-spur-gear",
                    "params": {
                        "prefix": "g1",
                        "module": 2.0,
                        "teeth": 16,
                        "pressure_angle": 20.0,
                        "face_width": 10.0,
                        "bore_diameter": 6.0,
                        "profile_shift": 0.0,
                        "backlash": 0.05,
                        "body_name": "Spur1"
                    }
                }),
            )
            .expect("parametric-spur-gear should succeed");
        assert_eq!(res["result"]["mode"], json!("parametric-spur-gear"));

        let (min0, max0) = body_bbox(&mut session, "Spur1");
        let width_x0 = max0[0] - min0[0];
        let height_z0 = max0[2] - min0[2];
        assert!((width_x0 - 36.0).abs() < 0.3, "expected ~36 mm OD, got {width_x0}");
        assert!((height_z0 - 10.0).abs() < 1e-3, "expected 10 mm face width, got {height_z0}");

        // 2. Edit g1_z (16 -> 24) and g1_fw (10 -> 18) in SolidCraft's native Parameters table
        //    WITHOUT re-running the plugin! Verify the kernel auto-rebuilds OD -> 52 mm & Z -> 18 mm.
        session
            .execute("parameters.change", &json!({ "name": "g1_z", "expression": "24" }))
            .expect("change g1_z");
        session
            .execute("parameters.change", &json!({ "name": "g1_fw", "expression": "18" }))
            .expect("change g1_fw");

        let (min1, max1) = body_bbox(&mut session, "Spur1");
        let width_x1 = max1[0] - min1[0];
        let height_z1 = max1[2] - min1[2];
        assert!((width_x1 - 52.0).abs() < 0.3, "expected auto-rebuilt ~52 mm OD, got {width_x1}");
        assert!((height_z1 - 18.0).abs() < 1e-3, "expected auto-rebuilt 18 mm face width, got {height_z1}");
    }

    #[test]
    fn parametric_parallel_pair_auto_rebuilds_center_distance_and_width() {
        let mut session = Session::default();
        let (pkg_dir, approval) = dist_and_approval(&mut session);

        // 1. Run parallel pair: m=2, z1=14, z2=22, fw=8 -> cd = 2*(14+22)/2 = 36 mm
        //    Gear 2 OD = 2*(22+2) = 48 mm -> X center at 36 mm -> X in [12, 60]
        let res: Value = session
            .execute(
                "plugin.run",
                &json!({
                    "path": pkg_dir.to_string_lossy(),
                    "approval": approval,
                    "entry": "parametric-parallel-pair",
                    "params": {
                        "prefix": "p1",
                        "module": 2.0,
                        "teeth_1": 14,
                        "teeth_2": 22,
                        "pressure_angle": 20.0,
                        "face_width": 8.0,
                        "bore_1": 5.0,
                        "bore_2": 8.0,
                        "backlash": 0.05
                    }
                }),
            )
            .expect("parametric-parallel-pair should succeed");
        assert_eq!(res["result"]["center_distance_mm"], json!(36.0));

        let (g2_min0, g2_max0) = body_bbox(&mut session, "p1_Gear");
        let g2_cx0 = 0.5 * (g2_min0[0] + g2_max0[0]);
        assert!((g2_cx0 - 36.0).abs() < 0.3, "expected Gear 2 center at X=36 mm, got {g2_cx0}");

        // 2. Change p1_z2 (22 -> 30) and p1_fw (8 -> 14) via Parameters table!
        //    New center distance = 2*(14+30)/2 = 44 mm; both gears widen to 14 mm!
        session
            .execute("parameters.change", &json!({ "name": "p1_z2", "expression": "30" }))
            .expect("change p1_z2");
        session
            .execute("parameters.change", &json!({ "name": "p1_fw", "expression": "14" }))
            .expect("change p1_fw");

        let (g1_min1, g1_max1) = body_bbox(&mut session, "p1_Pinion");
        let (g2_min1, g2_max1) = body_bbox(&mut session, "p1_Gear");
        let g2_cx1 = 0.5 * (g2_min1[0] + g2_max1[0]);
        assert!((g2_cx1 - 44.0).abs() < 0.3, "expected auto-rebuilt Gear 2 center at X=44 mm, got {g2_cx1}");
        assert!((g1_max1[2] - g1_min1[2] - 14.0).abs() < 1e-3, "expected Pinion Z=14 mm");
        assert!((g2_max1[2] - g2_min1[2] - 14.0).abs() < 1e-3, "expected Gear Z=14 mm");
    }
}

# GM Gear Suite — Developer & Agent Maintenance Guide

**Repository:** https://github.com/GaryMoFS/solidcraft-gear-suite (Public)  
**Local Working Directory:** `C:\Users\garym\Documents\gm gear suite`  
**Installed Plugin Directory:** `%LOCALAPPDATA%\SolidCraft\plugins\solidcraft-gear-suite\`  
**Current Version:** `v0.8.2` (SolidCraft `v0.48.1+`, WIT `craft:plugin@0.2.0`)

---

## 1. What This Plugin Provides

**GM Gear Suite** (`id: solidcraft-gear-suite`) is a `#![no_std]` WebAssembly Component (`wasm32-unknown-unknown`) plugin for SolidCraft that registers **10 parametric gear & mechanism generators** under the `Plugins` ribbon tab (`GM Gear ▾`):

| # | Command Entry | Module | Highlights |
|---|---|---|---|
| 1 | `gm-cylindrical-gear` | `plugin/src/cylindrical.rs` | External & Internal Ring; Straight, Helical, Herringbone; ISO Normal/Transverse; Module/DP/CP; optional `0.3m` root fillets; smart material/load defaults |
| 2 | `gm-parallel-pair` | `plugin/src/cylindrical.rs` | Straight/Helical/Herringbone, opposite hands, `By Teeth` / `By Ratio`, ISO 21771 profile shifts & exact shifted center distance $a_w$, `Tooth Thinning` / `Center Distance Increase` backlash, optional `joint.gear` |
| 3 | `gm-gear-train` | `plugin/src/train.rs` | 2–4 stage compound gear train; balanced ratio splitter; **Folded (100% coaxial $Z_{\text{sum}}$)** or **Linear** shafts; stage-by-stage torque scaling & stress check; optional `joint.gear` + rigid compound shaft locks |
| 4 | `gm-planetary` | `plugin/src/planetary.rs` | Sun + $N$ Planets + Internal Ring; `By Teeth` / `By Ratio` with automatic $(Z_s + Z_r) \bmod N = 0$ solver |
| 5 | `gm-rack-pinion` | `plugin/src/rack.rs` | Pinion + ISO 53 trapezoidal Linear Rack; `By Teeth` or `By Rack Length (mm)`; optional `joint.rack_pinion` |
| 6 | `gm-cycloidal` | `plugin/src/cycloidal.rs` | Exact equidistant epitrochoid disc (`sketch.equation_curve`), output pin holes, $N+1$ stationary ring pins |
| 7 | `gm-bevel-pair` | `plugin/src/bevel.rs` | Straight & 4-section logarithmic-spiral tapered loft **Spiral Bevel** (opposite hands), `30°–150°` shaft angle |
| 8 | `gm-worm-drive` | `plugin/src/worm.rs` | 1–4 start Worm + mating Helical Worm Wheel at 90° crossing axes |
| 9 | `gm-knurl` | `plugin/src/knurl.rs` | Straight, Diagonal, 3D Diamond knurl; auto-sizes and `solid.combine` intersects onto picked cylindrical `target_face` |
| 10 | `gm-geneva` | `plugin/src/geneva.rs` | 3–12 station Geneva Wheel (with locking crescents) + Drive Crank, Pin & Locking Hub |

---

## 2. Repository Structure

```text
C:\Users\garym\Documents\gm gear suite\
├── GUIDE.md                         # This guide
├── README.md                        # User-facing overview
├── plugin.json                      # Manifest + JSON schemas (MUST stay < 64 KiB, compact JSON)
├── wit/plugin.wit                   # craft:plugin@0.2.0 WIT (world: command-dialog-plugin)
├── patches/
│   └── plugin-dialog-tabs-r48.patch # Optional SolidCraft host UI patch for x-solidcraft-tab
├── packager/                        # Wraps core WASM into a WIT component + zip
│   ├── Cargo.toml
│   └── src/main.rs
├── plugin/                          # Core no_std Rust -> wasm32-unknown-unknown crate
│   ├── Cargo.toml
│   └── src/
│       ├── lib.rs                   # EntryGuest + DialogGuest, progress reporting, command helpers
│       ├── solver.rs                # Pure ISO 54 / ISO 21771 / Lewis bending / Hertz contact math
│       ├── panel.rs                 # dialog@0.1 smart defaults & live hints hook (no docked panel)
│       ├── placement.rs             # 3D PlacementFrame resolver (axis/plane/face/edge/point/base_plane)
│       ├── joints.rs                # component.from_bodies + joint.as_built + joint.gear / rack_pinion
│       ├── cylindrical.rs           # gm-cylindrical-gear & gm-parallel-pair + shared tooth builder
│       ├── train.rs                 # gm-gear-train (coaxial folded / linear multi-stage)
│       ├── planetary.rs             # gm-planetary
│       ├── rack.rs                  # gm-rack-pinion
│       ├── cycloidal.rs             # gm-cycloidal
│       ├── bevel.rs                 # gm-bevel-pair
│       ├── worm.rs                  # gm-worm-drive
│       ├── knurl.rs                 # gm-knurl
│       └── geneva.rs                # gm-geneva
└── dist/                            # Packaged outputs (gitignored or rebuilt by packager)
```

---

## 3. Build, Install & Verification (Windows GNU Toolchain)

This machine uses the **GNU Rust toolchain** (MSVC `link.exe` is not installed). Always set `RUSTUP_TOOLCHAIN` before running `cargo`:

```powershell
$env:RUSTUP_TOOLCHAIN = "stable-x86_64-pc-windows-gnu"

# 1. Build core WASM module
Set-Location "C:\Users\garym\Documents\gm gear suite\plugin"
cargo build --release --target wasm32-unknown-unknown

# 2. Embed WIT component metadata & package dist/
Set-Location "C:\Users\garym\Documents\gm gear suite\packager"
cargo run --release

# 3. Install into SolidCraft's local plugins folder
$dst = Join-Path $env:LOCALAPPDATA "SolidCraft\plugins\solidcraft-gear-suite"
New-Item -ItemType Directory -Force -Path $dst | Out-Null
Copy-Item "C:\Users\garym\Documents\gm gear suite\dist\solidcraft-gear-suite\*" $dst -Force
```

### Automated Verification via `solidcraft-cli` (`v0.48.1+`)

```powershell
$cli = "C:\Users\garym\Documents\SolidCraft-r48-trial\target\ci\solidcraft-cli.exe"
$dst = Join-Path $env:LOCALAPPDATA "SolidCraft\plugins\solidcraft-gear-suite"

# Validate manifest, permissions, limits, and WIT world
& $cli plugin check $dst

# Execute all entries against the real SolidCraft host
& $cli plugin test $dst --json
```

> **Tip:** To run targeted custom test cases without polluting the installed plugin folder, copy `dist\solidcraft-gear-suite\*` into a temporary folder, add `tests\<case>.json` files (`{"entry":"...","params":{...}}` written as **UTF-8 without BOM**), and run `& $cli plugin test <temp-dir> --json --out <out-dir>`.

> **Full-geometry check:** every entry's `fast_preview` defaults to `true`, so the built-in default cases only build preview envelopes. Before a release, also add one case per entry with `"fast_preview": false` (for `gm-knurl`, also pass its required `knurl_type`, `diameter`, `length`, `pitch` and `depth`, or the harness rejects the case).

> **Packager after a folder move:** the packager finds the repo through `CARGO_MANIFEST_DIR`, which is fixed at compile time. If the repo moves, `touch packager/src/main.rs` before `cargo run`.

> **WIT:** keep `wit/plugin.wit` byte-identical to the host's `crates/craft-plugin/wit/plugin.wit`. ACO's maintenance run flags any plugin whose WIT hash differs from a new SolidCraft trial.

---

## 4. SolidCraft Host Compatibility & Optional Dialog Tabs Patch

- **Capabilities:** `["commands@0.2", "dialog@0.1"]` (`world command-dialog-plugin`).
- **Works out-of-the-box on stock SolidCraft `v0.48.1+`** (if `x-solidcraft-tab` is not patched into the host, SolidCraft's dialog renderer ignores or rejects unknown annotations — **important:** stock SolidCraft `v0.48.1` has an `ANNOTATIONS` allowlist in `crates/ui-egui/src/plugin_menu.rs`, so apply `patches/plugin-dialog-tabs-r48.patch` when building a new SolidCraft release, or strip `x-solidcraft-tab` if targeting an unpatched stock build).
- **Never overwrite `target\ci\solidcraft.exe`** while a `solidcraft` desktop process is running.

---

## 5. Architecture & Coding Rules

1. **`#![no_std]` + Zero Panics:** `plugin/src/lib.rs` enforces `#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]`. Use `alloc::*`, `libm::*`, and `?` error propagation everywhere.
2. **All Geometry Must Stay Parametric:**
   - Register user/derived parameters via `upsert_param` with a unique prefix (`choose_unique_prefix` + `unique_name`) so repeated runs in the same document never collide.
   - Use 2D `sketch.pattern.circular` (`count: "<p>_z"`) + a single `solid.extrude` / `solid.sweep` (draw bore/ring circles **before** circular patterning). Never use 3D `solid.pattern.circular` for gear teeth (5–10× slower).
3. **`Fast Preview` Mode:**
   - Every command reads `crate::is_preview(raw)` (which checks both `_preview` and `fast_preview`).
   - When `true`, build lightweight cylinder/pitch-circle envelopes (~50–250 ms). When `false` (Run), build full 3D teeth and optional assembly joints.
4. **Smart Dialog Hook (`plugin/src/panel.rs`):**
   - `dialog@0.1` (`update_fields`) runs read-only on a 120 ms debounce.
   - Return `{"<field>": {"default": ..., "hint": "..."}}`. SolidCraft's host **never overwrites** a field the user has manually edited.
   - Keep `hint` strings short so they never overrun the dialog frame.
5. **Manifest Size Limit (`< 64 KiB`):**
   - `plugin.json` MUST remain under 64 KiB (currently ~46 KiB compact JSON). Keep it minified (`json.dumps(..., separators=(",", ":"))`) and keep descriptions/tab names concise (`Geometry`, `Profile`, `Material`, `Placement`).

---

## 6. Critical Engineering & SolidCraft Gotchas

| Gotcha | Rule / Solution |
|---|---|
| **Angle units in expressions** | Inside `sketch.equation_curve`, `cos(t)` expects **radians**. Outside `sketch.equation_curve` (`sketch.arc.three_point`, `sketch.line`), `cos(x)` treats unitless numbers as **degrees** unless suffixed with `* 1 rad`. |
| **PowerShell UTF-8 BOM** | `Set-Content -Encoding UTF8` writes a BOM that `solidcraft-cli` rejects. Always use `[System.IO.File]::WriteAllText($p, $s, (New-Object System.Text.UTF8Encoding($false)))`. |
| **`sketch.fillet` on tooth roots** | Curves must share exact vertex IDs (`<curve>.end` / `bot_start`), fillet radius must be capped to `<= 0.75 * min(radial_len, root_arc_len)`, and `sketch.fillet` only runs when `needs_radial == true` ($r_f < r_b$, arc-line corner) — SolidCraft `v0.48.1` does not yet fillet `equation_curve` fit-point splines directly. |
| **Folded multi-stage coaxial shafts** | In `train.rs`, `coaxial_folded == true` constrains ALL stages to the exact same tooth sum $z_{p,i} + z_{g,i} = Z_{\text{sum}}$ so Shaft A (`0.0 mm`) and Shaft B (`+cd mm`) align with 0.000 mm error across all stages. |
| **Folded odd-tooth pinion meshing** | When `dir_sign < 0` (folded back toward `-X`), always set pinion `phi_p = pi` (tooth #0 pointing `-X`) and gear `phi_g = pi / zg` (space facing `+X`). Never leave `phi_p = None`, because odd-tooth pinions have a space at `-X`. |
| **`x-solidcraft-visible-when`** | Hidden conditional fields are stripped from Run/Preview params. Never put a `required` field behind `x-solidcraft-visible-when`, and never use it to fake tabs (use `x-solidcraft-tab`). |
| **`sketch.arc.three_point`** | The middle point field is `"through"` (not `"mid"`). |

---

## 7. How to Add a New Gear / Mechanism Command

1. Add `plugin/src/<new_mech>.rs` with `#[derive(Deserialize)] #[serde(default)]` input struct, `resolve_placement`, unique prefixing, `crate::report_progress`, and `Fast Preview` support.
2. Register `mod <new_mech>;` and match `"gm-<new-mech>"` in `plugin/src/lib.rs`.
3. Optionally add smart defaults/hints in `plugin/src/panel.rs` (`compute_dialog_hints`).
4. Add the entry to `plugin.json` with `x-solidcraft-tab` tabs (`Geometry`, `Profile`, `Material`, `Placement`), keeping `fast_preview` untabbed at the bottom and compact `plugin.json < 64 KiB`.
5. Build WASM, run `packager`, install to `%LOCALAPPDATA%\SolidCraft\plugins\solidcraft-gear-suite\`, and verify with `solidcraft-cli plugin check` and `solidcraft-cli plugin test`.
6. Commit to `dev/parametric-gears`, fast-forward `main`, and push to GitHub.

---

## 8. Future Roadmap Ideas (`v0.9.0+`)

- Expand Lewis/Hertz stress checks & `create_joints` to **Planetary**, **Bevel**, and **Worm** drives.
- Add **internal gear** support to `GM Parallel Pair` (external pinion inside internal ring with `joint.gear` `internal: true`).
- Add **custom shaft / keyway / hub** options on cylindrical & train gears.
- Add **STEP / BOM export helper** or engineering summary note written to document metadata.

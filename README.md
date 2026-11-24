# SolidCraft Gear Suite (Parametric Equation-Driven)

A Shwivel-inspired parametric gear plugin suite for **SolidCraft r47+** (`craft:plugin@0.2.0`, `commands@0.2`).

Unlike static gear generators that bake numeric coordinates, every gear created by this plugin publishes named **Design Parameters** and builds native expression-driven features (`sketch.equation_curve`, `solid.extrude`, `solid.pattern.circular`, `solid.cylinder`). Editing tooth count, module, pressure angle, profile shift, backlash, or face width in SolidCraft's native **Parameters** table automatically rebuilds the involute flanks, tooth count, gear width, and mating shaft center distance in place.

## Entries
- **`parametric-spur-gear`**: External involute spur gear with profile shift, backlash, and optional bore.
- **`parametric-parallel-pair`**: Meshing parallel spur gear pair with auto-rebuilding center distance and tooth phase alignment.

## Build & Test (Windows GNU Toolchain)
```powershell
$env:RUSTUP_TOOLCHAIN = "stable-x86_64-pc-windows-gnu"
cd plugin; cargo build --release --target wasm32-unknown-unknown
cd ..\packager; cargo run --release
cargo test --profile ci
```

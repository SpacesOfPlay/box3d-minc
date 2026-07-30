# box3d-minc

A [minc](https://minc.dev)-language port of
[Box3D](https://github.com/erincatto/box3d), Erin Catto's 3D rigid-body
physics engine, transpiled from the C sources. Ships with a port of the
upstream sample browser: orbit/fly camera, mouse grab, Dear ImGui
overlay, procedural sky. Runs natively (Win/macOS/Linux) or in the browser
(WebAssembly + WebGL2).

Live demo of wasm build: [sample browser](https://spacesofplay.github.io/box3d-minc/)

## Quickstart

install minc if needed:
```
# Windows
powershell -c "irm minc.dev/install.ps1 | iex"

# macOS / Linux
curl -fsSL https://minc.dev/install | bash
```

clone repo:
```powershell
git clone https://github.com/SpacesOfPlay/box3d-minc
cd box3d-minc
./build.ps1                 # builds + runs the sample browser
```

Run it in the browser instead:

```powershell
./build.ps1 wasm            # builds, serves, opens the browser
```

## Sample browser controls

- `[` / `]` — previous / next sample; menu bar has the full catalogue
- `P` pause, `O` single-step (+Shift: 5), `R` restart
- Left-click select, Ctrl+drag grab a body, Shift+click launch a ball
- Alt+left-drag orbit, Alt+middle pan, Alt+right radial zoom,
  scroll zoom; right-drag + WASD fly
- `F` frame the scene, `M` metrics drawer, `Tab` hide UI, SPACE launch

## Using the physics from your own code

The physics library is a plain-C-shaped API (`b3CreateWorld`,
`b3World_Step`, `b3CreateBody`, ...). Import it and build:

```minc
import box3d;

void main() {
    b3WorldDef def = b3DefaultWorldDef();
    b3WorldId world = b3CreateWorld(&def);
    // ...
}
```

```powershell
./build.ps1 run my_sim.mc
```

`import box3d;` resolves from `lib/` when you build from this
directory.

The sample browser also imports `sokol_all`, `sokol_imgui`, `imgui`,
`math`, `str` and `linear`. Those are part of the minc standard
library. The compiler resolves them from `lib/` beside the minc
binary.

## Cross-platform

One version serves every target. The Box3D runtime
(`lib/box3d_ext.mc`) is written over minc's cross-platform threading
and timing builtins, so `./build.ps1` builds a native binary on
Windows (D3D11), Linux and macOS, and `./build.ps1 wasm` builds the
browser target (WebGL2) from any host. Worker threads run on all three
native targets. `./build.ps1 wasm` is single-threaded; add `--threads`
to the minc call for the parallel wasm build (`live-demo/README.md`).

## Layout

```
lib/              box3d.mc (physics) + box3d_ext.mc (runtime).
sample_browser/   the samples port
  main.mc           entry point, event routing
  sample.mc         sample registry, picking, launcher
  camera.mc         orbit + fly camera
  gui.mc            Dear ImGui overlay
  renderer.mc       shaders, meshes, draw calls
  sample_*.mc       the scenes, by category
build/            output (gitignored)
```

## Licenses

See `LICENSE.md` — Box3D (MIT), plus the sokol (zlib), Dear ImGui (MIT)
and minc-compiler notices for the standard-library modules the sample
browser imports.

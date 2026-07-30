# live-demo

The sample browser compiled to wasm, as a static page.

| file | source |
| --- | --- |
| `index.html`, `bench.html` | this directory |
| `sokol_wasm_host.js`, `wasm_threads_host.js`, `coi-serviceworker.js` | the minc install (`apps/`), copied in at publish |
| `box3d_bench.mc` | the minc install (`apps/`), copied in at publish |
| `data/` | the samples' meshes and trees |
| `*.wasm` | built by `.github/workflows/pages.yml` — gitignored |

Two pages: `index.html` is the sample browser, `bench.html` runs 1080
headless steps of the box3d demo scene and prints a checksum.

## Threads

Each program ships twice, `x.wasm` and `x.threads.wasm`. The threaded
build needs cross-origin isolation. The script `coi-serviceworker.js` 
adds them with a service worker. First load reloads once; after that 
the loader picks the threaded build and the Workers slider is live. 
Fails in private mode and falls back to single-threaded.

## Switching threads off

Some samples currently do not work correct on the threaded build. Turn 
off wasm threads with the button in bottom right corner.

The override is read from the URL on load, not from the click, so it
still takes when the threaded build has wedged the page.

## Build the wasm

From the repo root:

    minc sample_browser/main.mc --target wasm -o live-demo/sample_browser.wasm
    minc sample_browser/main.mc --target wasm --threads -o live-demo/sample_browser.threads.wasm
    minc live-demo/box3d_bench.mc --target wasm -o live-demo/box3d_bench.wasm
    minc live-demo/box3d_bench.mc --target wasm --threads -o live-demo/box3d_bench.threads.wasm

## Run it locally

From this directory:

    python -m http.server 8000

then open <http://localhost:8000/>. A `file://` open will not work.
`localhost` counts as a secure context, so threads work there too.

## Performance

The large scenes — Convex Pile, Junkyard, Large World — are slow with
threads off, since physics then runs on one thread where the native
build uses several. Even the threaded wasm runs much slower than native
builds.

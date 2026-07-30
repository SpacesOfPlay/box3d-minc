#!/usr/bin/env bash
# build.sh — build (and run) a box3d-minc program. Twin of build.ps1.
#
#   ./build.sh                      # build + run the sample browser
#   ./build.sh run <file.mc>        # build + run your own program
#   ./build.sh wasm [<file.mc>]     # build + serve in the browser
#   ./build.sh build [<file.mc>]    # compile only
#   ./build.sh clean
#
# One transpile serves every target: the Box3D platform layer
# (lib/box3d_platform.mc) rides minc's cross-platform threading/timing
# builtins, so the native build runs on Windows/Linux/macOS and
# `./build.sh wasm` targets the browser.

set -euo pipefail
root="$(cd "$(dirname "$0")" && pwd)"

cmd="${1:-run}"
src="${2:-}"
case "$cmd" in
    *.mc) src="$cmd"; cmd=run ;;
esac

if [ "$cmd" = clean ]; then
    rm -rf "$root/build"
    echo clean.
    exit 0
fi

# minc: $MINC override (install dir, or a direct binary path), else
# PATH (installed toolchain), else next
# to this script (manual zip layout). Install from https://minc.dev.
if [ -n "${MINC:-}" ]; then
    if [ -d "$MINC" ]; then minc="$MINC/minc"; else minc="$MINC"; fi
elif command -v minc >/dev/null 2>&1; then
    minc="$(command -v minc)"
else
    minc="$root/minc"
fi
if [ ! -x "$minc" ]; then
    echo "minc compiler not found. Install it:" >&2
    echo "  curl -fsSL https://minc.dev/install | bash" >&2
    echo "or set MINC (see install_minc.md)." >&2
    exit 1
fi

[ -n "$src" ] || src="$root/sample_browser/main.mc"
case "$src" in
    /*) ;;
    *) src="$root/$src" ;;
esac
[ -d "$src" ] && src="$src/main.mc"
[ -f "$src" ] || { echo "no such file: $src" >&2; exit 1; }
name="$(basename "$src" .mc)"
[ "$name" = main ] && name="$(basename "$(dirname "$src")")"

[ -f "$root/lib/box3d.mc" ] || { echo "missing lib/box3d.mc" >&2; exit 1; }

mkdir -p "$root/build"

cd "$root"   # `import box3d;` resolves against ./lib from here
if [ "$cmd" = wasm ]; then
    mkdir -p "$root/build/web"
    exec "$minc" run --target wasm "$src" -o "$root/build/web/$name.wasm"
fi

exe="$root/build/$name"
"$minc" "$src" -o "$exe"
echo "built $exe"
if [ "$cmd" = run ]; then exec "$exe"; fi

# Licenses

This repository redistributes one upstream project: **Box3D**
(`lib/box3d.mc` and its runtime, plus the samples port). The
standard-library modules the sample browser imports (sokol, Dear ImGui,
math, ...) are NOT part of this repository — the minc compiler resolves
them from its own installation. Their licenses are reproduced below
because the built program links them.

## Box3D (MIT)

Upstream: https://github.com/erincatto/box3d — the pinned commit is
recorded in `lib/UPSTREAM_COMMIT.txt`. `lib/box3d.mc` is a
machine transpile of the Box3D C sources; `sample_browser/`
is a hand-port of upstream's `samples/` application (scene setups,
solver parameters, camera, GUI layout, sky and grid shaders).

```
MIT License

Copyright (c) 2025 Erin Catto

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## sokol (zlib/libpng)

Upstream: https://github.com/floooh/sokol. `sokol_all` and
`sokol_imgui` (minc ports of sokol_app / sokol_gfx / sokol_glue /
sokol_imgui, provided by the minc install) inherit sokol's
zlib/libpng license:

```
Copyright (c) 2018 Andre Weissflog

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

    1. The origin of this software must not be misrepresented; you
       must not claim that you wrote the original software. If you
       use this software in a product, an acknowledgment in the
       product documentation would be appreciated but is not
       required.

    2. Altered source versions must be plainly marked as such, and
       must not be misrepresented as being the original software.

    3. This notice may not be removed or altered from any source
       distribution.
```

## Dear ImGui (MIT)

Upstream: https://github.com/ocornut/imgui. `imgui` (a minc port of
the Dear ImGui core via the dear_bindings C API, also MIT, provided by
the minc install) is linked by the sample browser's overlay.

```
The MIT License (MIT)

Copyright (c) 2014-2025 Omar Cornut

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## Box3D minc framework / support code (MIT)

Any new files or manual edits added to this repo to support the Box3D
minc distribution and the examples here are also MIT license.

```
MIT License

Copyright (c) 2025 Mattias Ljungström, Spaces of Play UG (haftungsbeschrankt)

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

## minc compiler (closed-source)

The minc compiler binary is **not** under any of the above licenses —
it's a separate closed-source binary from https://minc.dev. Install it
from https://minc.dev (see install_minc.md). Using this distribution
requires accepting minc's own license. The minc binary itself is not
redistributed inside this repository.

## transminc (transpiler, closed-source)

`lib/box3d.mc` is generated by transminc from the Box3D sources and 
inherits Box3D's MIT license above. The runtime support in 
`lib/box3d_ext.mc` is part of the minc/transminc toolchain, provided
under the same terms as this repository. The standard-library modules
and web host files the program also uses live in the minc installation,
not in this repository.

// minc --threads browser host: picks app.threads.wasm when the page is
// cross-origin isolated, falls back to app.wasm otherwise.
(() => {
'use strict';

function detect() {
    const q = new URLSearchParams(location.search).get('threads');
    if (q === '0') return { ok: false, reason: 'forced off (?threads=0)' };
    if (typeof crossOriginIsolated !== 'undefined' && !crossOriginIsolated && q !== '1')
        return { ok: false, reason: 'page not cross-origin isolated' };
    if (typeof SharedArrayBuffer === 'undefined')
        return { ok: false, reason: 'no SharedArrayBuffer' };
    try { new WebAssembly.Memory({ initial: 1, maximum: 1, shared: true }); }
    catch (e) { return { ok: false, reason: 'shared wasm memory rejected' }; }
    return { ok: true, reason: 'cross-origin isolated' };
}

function makeMath() {
    return new Proxy({
        sin: Math.sin, cos: Math.cos, tan: Math.tan, sqrt: Math.sqrt,
        asin: Math.asin, acos: Math.acos, atan: Math.atan, atan2: Math.atan2,
        exp: Math.exp, log: Math.log, log2: Math.log2, log10: Math.log10,
        pow: Math.pow, fmod: (a, b) => a % b, fabs: Math.abs,
        floor: Math.floor, ceil: Math.ceil, round: Math.round,
        sinf: Math.sin, cosf: Math.cos, tanf: Math.tan, sqrtf: Math.sqrt,
        asinf: Math.asin, acosf: Math.acos, atanf: Math.atan, atan2f: Math.atan2,
        expf: Math.exp, logf: Math.log, powf: Math.pow, fmodf: (a, b) => a % b,
        fabsf: Math.abs, floorf: Math.floor, ceilf: Math.ceil, roundf: Math.round,
    }, { get: (t, k) => (k in t ? t[k] : (x) => x) });
}

function makeEnv(getMemory, onWrite, extra) {
    return new Proxy({
        write: (fd, ptr, len) => {
            const m = new Uint8Array(getMemory().buffer);
            let s = '';
            for (let i = 0; i < Number(len); i++) s += String.fromCharCode(m[Number(ptr) + i]);
            onWrite(s, Number(fd));
            return len;
        },
        clock: () => BigInt(Math.round(performance.now() * 1e6)),
        __sys_exit: (c) => { throw { mincExit: Number(c) }; },
        __minc_cpu_count_host: () => BigInt(navigator.hardwareConcurrency || 1),
        ...extra,
    }, { get: (t, k) => (k in t ? t[k] : (...a) => 0n) });
}

// Pool workers park on Atomics command slots: the main thread may spin
// inside wasm without yielding, so nothing after startup can depend on
// the event loop.
const WORKER_SRC = `
'use strict';
const makeMath = ${makeMath.toString()};
const makeEnv = ${makeEnv.toString()};
onmessage = async (e) => {
    const { module, memory, ctrl, ctrlOff } = e.data;
    const env = makeEnv(() => memory, (s) => console.log('[worker]', s), { memory });
    const inst = await WebAssembly.instantiate(module, { env, math: makeMath() });
    const i32v = new Int32Array(ctrl, ctrlOff, 16);
    const i64v = new BigInt64Array(ctrl, ctrlOff, 8);
    postMessage('ready');
    for (;;) {
        Atomics.wait(i32v, 0, 0);
        inst.exports.__stack_pointer.value = Number(i64v[4]);
        inst.exports.__minc_thread_entry(i64v[1], i64v[2], i64v[3]);
        if (i64v[5] && new Int32Array(memory.buffer, Number(i64v[5]), 1)[0] !== 0x7C0FFEE5)
            console.error('minc: worker stack overflow detected (canary smashed)');
        Atomics.store(i32v, 0, 0);
        Atomics.notify(i32v, 0);
    }
};
`;

async function load(base, opts) {
    const o = opts || {};
    const onWrite = o.onWrite || ((s) => console.log(s));
    const pool = o.pool || 8;
    const det = detect();

    if (det.ok) {
        try {
            return await loadThreads(base + '.threads.wasm', onWrite, pool);
        } catch (e) {
            console.warn('threads build failed, falling back:', e.message || e);
        }
    }
    const bytes = await (await fetch(base + '.wasm')).arrayBuffer();
    let inst;
    const env = makeEnv(() => inst.exports.memory, onWrite, {});
    inst = (await WebAssembly.instantiate(bytes, { env, math: makeMath() })).instance;
    return { exports: inst.exports, threads: false, workers: 0, reason: det.reason };
}

async function loadThreads(url, onWrite, pool) {
    const bytes = await (await fetch(url)).arrayBuffer();
    const module = await WebAssembly.compile(bytes);
    // Import limits are not introspectable; grow the initial guess until
    // instantiation stops failing on "memory import ... smaller than".
    let memory = null, inst = null, err = null;
    const STACK = 2097152;
    const workers = [];
    const views = [];
    const ctrl = new SharedArrayBuffer(pool * 64);
    let extra = {
        __minc_thread_create: (entry, arg) => {
            const flag = Number(inst.exports.__wasm_alloc(16n));
            new Int32Array(memory.buffer, flag, 1)[0] = 0;
            for (;;) {
                for (let k = 0; k < workers.length; k++) {
                    const v = views[k];
                    if (Atomics.load(v.i32, 0) === 0) {
                        // One stack per slot, reused — the heap never frees.
                        if (!v.stack) v.stack = Number(inst.exports.__wasm_alloc(BigInt(STACK)));
                        new Int32Array(memory.buffer, v.stack, 1)[0] = 0x7C0FFEE5;  // canary
                        v.i64[1] = entry; v.i64[2] = arg; v.i64[3] = BigInt(flag);
                        v.i64[4] = BigInt((v.stack + STACK) & ~15);
                        v.i64[5] = BigInt(v.stack);
                        Atomics.store(v.i32, 0, 1);
                        Atomics.notify(v.i32, 0);
                        return BigInt(flag);
                    }
                }
            }
        },
    };
    for (const initial of [64, 256, 1024, 4096]) {
        try {
            try { memory = new WebAssembly.Memory({ initial, maximum: 32768, shared: true }); }
            catch (e2) { memory = new WebAssembly.Memory({ initial, maximum: 16384, shared: true }); }
            const env = makeEnv(() => memory, onWrite, { memory, ...extra });
            inst = await WebAssembly.instantiate(module, { env, math: makeMath() });
            err = null;
            break;
        } catch (e) { err = e; }
    }
    if (err) throw err;

    const blob = new Blob([WORKER_SRC], { type: 'text/javascript' });
    const blobUrl = URL.createObjectURL(blob);
    const ready = [];
    for (let k = 0; k < pool; k++) {
        const w = new Worker(blobUrl);
        w.onerror = (e) => console.error('pool worker:', e.message || e);
        w.postMessage({ module, memory, ctrl, ctrlOff: k * 64 });
        views.push({ i32: new Int32Array(ctrl, k * 64, 16), i64: new BigInt64Array(ctrl, k * 64, 8) });
        workers.push(w);
        ready.push(new Promise((res) => { w.onmessage = res; }));
    }
    await Promise.all(ready);
    inst.exports.__minc_init_data();
    return { exports: inst.exports, threads: true, workers: pool, memory, reason: 'cross-origin isolated' };
}

window.mincThreads = { detect, load };
})();

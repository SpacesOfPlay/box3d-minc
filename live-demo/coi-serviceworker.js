// COOP/COEP via service worker, for hosts that can't set headers
// (GitHub Pages). One file, two roles: page script registers it, the
// SW re-serves every response with the isolation headers. Scope = its
// directory, so pages above it are never affected.
if (typeof window === 'undefined') {
    self.addEventListener('install', () => self.skipWaiting());
    self.addEventListener('activate', (e) => e.waitUntil(self.clients.claim()));
    self.addEventListener('fetch', (e) => {
        const req = e.request;
        if (req.cache === 'only-if-cached' && req.mode !== 'same-origin') return;
        e.respondWith(fetch(req).then((r) => {
            if (r.status === 0) return r;
            const h = new Headers(r.headers);
            h.set('Cross-Origin-Embedder-Policy', 'require-corp');
            h.set('Cross-Origin-Opener-Policy', 'same-origin');
            return new Response(r.body, { status: r.status, statusText: r.statusText, headers: h });
        }).catch((err) => new Response(null, { status: 500, statusText: '' + err })));
    });
} else if (crossOriginIsolated) {
    sessionStorage.removeItem('coiReload');
} else if (window.isSecureContext && 'serviceWorker' in navigator &&
           new URLSearchParams(location.search).get('threads') !== '0') {
    // One reload after the SW activates picks up the injected headers.
    // The sessionStorage guard stops a loop if isolation still fails
    // (private mode etc.) — the page then runs single-threaded.
    navigator.serviceWorker.register(document.currentScript.src).then(() =>
        navigator.serviceWorker.ready
    ).then(() => {
        if (sessionStorage.getItem('coiReload')) return;
        sessionStorage.setItem('coiReload', '1');
        location.reload();
    }).catch(() => {});
}

// build.mc — build (and run) a box3d-minc program.
//
// Usage, from this folder:
//   minc run                 build + run the sample browser
//   minc run <file.mc>       build + run your own program
//   minc wasm [<file.mc>]    build + serve in the browser
//   minc wasm threads        also build the threaded wasm and serve both
//   minc wasm --no-run       serve without opening the browser
//   minc build [<file.mc>]   compile only
//   minc clean
//
// A program is ONE compilation unit: `import box3d;` pulls in the
// physics + runtime from lib/, and `import sokol_all;` etc. resolve
// against the minc install when building from this directory.
//
// The minc compiler is taken from MINC, then PATH, then this folder.
// Install minc from https://minc.dev.

@minc_min_version "0.9.11"

// minc 0.9.9 ignores the above tag, this will force an error.
// remove at some point in future.
when !defined(MINC_VERSION) || MINC_VERSION < 9011 {
    minc_0_9_10_or_newer_required please_update_minc;
}

import process;
import file;
import str;

when os(windows) { str EXE_SUFFIX = ".exe"; }
when os(linux) || os(macos) { str EXE_SUFFIX = ""; }

void out(str s) {
    write(stdout(), s.data, s.len);
    return;
}

void say(str s) {
    out(s);
    write(stdout(), "\n", 1);
    return;
}

// "<dir>/<name><ext>", without leaking the joined name.
string join_named(str dir, str name, str ext) {
    string base = str_concat(name, ext);
    defer free(base);
    return path_join(dir, str_from(base.data, base.len));
}

void die(str s) {
    write(stderr(), s.data, s.len);
    write(stderr(), "\n", 1);
    exit(1);
    return;
}

// MINC first (an install dir or the binary itself), then PATH, then a
// binary sitting next to this script.
string find_minc() {
    string env = env_get("MINC");
    if env.len > 0 {
        str e = str_from(env.data, env.len);
        if path_is_dir(e) {
            string cand = join_named(e, "minc", EXE_SUFFIX);
            free(env);
            return cand;
        }
        return env;
    }
    free(env);

    string onpath = path_which("minc");
    if onpath.len > 0 { return onpath; }
    free(onpath);

    string local = str_concat("./minc", EXE_SUFFIX);
    if path_exists(str_from(local.data, local.len)) { return local; }
    free(local);

    string none = { .data = null, .len = 0 };
    return none;
}

// A directory means "<dir>/main.mc"; anything else is taken as given.
string resolve_source(str arg) {
    if arg.len == 0 { return str_concat("sample_browser/main.mc", ""); }
    if path_is_dir(arg) { return path_join(arg, "main.mc"); }
    return str_concat(arg, "");
}

// Name the output after the program's folder when the file is a
// main.mc, so every sample does not build to "main".
string output_name(str src) {
    str stem = path_stem(src);
    if !str_equal(stem, "main") { return str_concat(stem, ""); }
    str parent = path_basename(path_dirname(src));
    if parent.len == 0 { return str_concat(stem, ""); }
    return str_concat(parent, "");
}

i32 main() {
    i32 argc = get_argc();
    str verb = "run";
    str target = "";
    bool no_run = false;
    bool threads = false;

    for i32 i = 1; i < argc; i++ {
        str a = str_from_cstr(get_arg(i));
        if str_equal(a, "--no-run") { no_run = true; }
        else if str_equal(a, "threads") { threads = true; }
        else if i == 1 {
            // A .mc path in the verb slot means "run this".
            if str_ends_with(a, ".mc") { target = a; }
            else { verb = a; }
        } else if target.len == 0 { target = a; }
    }

    if str_equal(verb, "clean") {
        ignore dir_remove("build");
        say("clean.");
        return 0;
    }

    string minc = find_minc();
    defer free(minc);
    if minc.len == 0 {
        say("");
        say("minc compiler not found.");
        say("Install it:  powershell -c \"irm minc.dev/install.ps1 | iex\"");
        say("or set MINC (see install_minc.md).");
        die("See README.md (Quickstart) and LICENSE.md.");
    }
    str cc = str_from(minc.data, minc.len);

    if !path_exists("lib/box3d.mc") {
        die("missing lib/box3d.mc — dist is incomplete");
    }

    string src = resolve_source(target);
    defer free(src);
    str srcp = str_from(src.data, src.len);
    if !path_exists(srcp) {
        write(stderr(), "no such file: ", 14);
        die(srcp);
    }
    string name = output_name(srcp);
    defer free(name);
    str namep = str_from(name.data, name.len);

    ignore dir_create("build");

    i32 rc = 0;

    if str_equal(verb, "wasm") {
        ignore dir_create("build/web");
        string wasm_out = join_named("build/web", namep, ".wasm");
        defer free(wasm_out);

        // The threaded build is a second artifact beside the serial one.
        // The browser host loads it only on a cross-origin-isolated page
        // and falls back to the serial build otherwise, so both have to
        // be there. minc serves the isolation headers once it sees this
        // file staged.
        if threads {
            string thr_out = join_named("build/web", namep, ".threads.wasm");
            defer free(thr_out);
            out("building ");
            out(namep);
            say(" with threads...");
            ProcCmd tc = { .args = {
                cc, srcp, "--target", "wasm", "--threads",
                "-o", str_from(thr_out.data, thr_out.len)
            } };
            ProcResult tr = proc_run(&tc);
            i32 trc = tr.exit_code;
            proc_result_free(&tr);
            if trc != 0 { die("threads build failed"); }
        }

        out("building + serving ");
        out(namep);
        say(" for the web (wasm)...");
        ProcCmd c = { .args = {
            cc, "run", "--target", "wasm", srcp,
            "-o", str_from(wasm_out.data, wasm_out.len)
        } };
        if no_run { proc_arg(&c, "--no-browser"); }
        ProcResult r = proc_run(&c);
        rc = r.exit_code;
        proc_result_free(&r);
    } else {
        string exe = join_named("build", namep, EXE_SUFFIX);
        defer free(exe);
        str exep = str_from(exe.data, exe.len);
        out("building ");
        say(namep);
        ProcCmd c = { .args = { cc, srcp, "-o", exep } };
        ProcResult r = proc_run(&c);
        rc = r.exit_code;
        proc_result_free(&r);
        if rc != 0 || !path_exists(exep) { die("minc compile failed"); }
        out("built ");
        say(exep);
        if str_equal(verb, "run") && !no_run {
            ProcCmd run = { .args = { exep } };
            ProcResult rr = proc_run(&run);
            rc = rr.exit_code;
            proc_result_free(&rr);
        }
    }
    return rc;
}

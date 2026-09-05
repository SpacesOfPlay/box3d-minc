// build.mc - build (and run) a box3d-minc program.
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
// physics + runtime from lib/, and `import sokol_all;` resolves
// against the minc install when building from this folder.
//
// The compiler is taken from MINC, then PATH, then this folder
// (install: https://minc.dev).

@minc_min_version "0.9.14"

// Older minc ignores the tag above; this forces an error instead.
when !defined(MINC_VERSION) || MINC_VERSION < 9014 {
    minc_0_9_14_or_newer_required please_update_minc;
}

import process;
import file;
import str;

when os(windows) { str EXE_SUFFIX = ".exe"; }
when os(linux) || os(macos) { str EXE_SUFFIX = ""; }

string join_named(str dir, str name, str ext) {
    string base = str_concat(name, ext);
    defer free(base);
    return path_join(dir, base);
}

void die(str s) {
    eprint("{}\n", s);
    exit(1);
    return;
}

// MINC (install dir or binary), then PATH, then this folder.
string find_minc() {
    string env = env_get("MINC");
    if env.len > 0 {
        if path_is_dir(env) {
            string cand = join_named(env, "minc", EXE_SUFFIX);
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
    if path_exists(local) { return local; }
    free(local);

    string none = { .data = null, .len = 0 };
    return none;
}

// A directory means "<dir>/main.mc"; anything else is taken as given.
string resolve_source(str arg) {
    if arg.len == 0 { return str_concat("sample_browser/main.mc", ""); }
    if path_is_dir(arg) { return path_join(arg, "main.mc"); }
    return string(arg);
}

// A main.mc takes its name from the parent folder, so samples do not
// all build to "main".
string output_name(str src) {
    str stem = path_stem(src);
    if !str_equal(stem, "main") { return string(stem); }
    str parent = path_basename(path_dirname(src));
    if parent.len == 0 { return string(stem); }
    return string(parent);
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
        print("clean.\n");
        return 0;
    }

    string minc = find_minc();
    defer free(minc);
    if minc.len == 0 {
        print("\nminc compiler not found.\n"
              "Install it:  powershell -c \"irm minc.dev/install.ps1 | iex\"\n"
              "or set MINC (see install_minc.md).\n");
        die("See README.md (Quickstart) and LICENSE.md.");
    }

    if !path_exists("lib/box3d.mc") {
        die("missing lib/box3d.mc - dist is incomplete");
    }

    string src = resolve_source(target);
    defer free(src);
    if !path_exists(src) {
        eprint("no such file: {}\n", src);
        exit(1);
    }
    string name = output_name(src);
    defer free(name);

    ignore dir_create("build");

    i32 rc = 0;

    if str_equal(verb, "wasm") {
        ignore dir_create("build/web");
        string wasm_out = join_named("build/web", name, ".wasm");
        defer free(wasm_out);

        // A second artifact beside the serial one: the browser host
        // loads it only on a cross-origin-isolated page. minc serves
        // the isolation headers once it sees the file staged.
        if threads {
            string thr_out = join_named("build/web", name, ".threads.wasm");
            defer free(thr_out);
            print("building {} with threads...\n", name);
            ProcCmd tc = { .args = {
                minc, src, "--target", "wasm", "--threads",
                "-o", thr_out
            } };
            ProcResult tr = proc_run(&tc);
            i32 trc = tr.exit_code;
            proc_result_free(&tr);
            if trc != 0 { die("threads build failed"); }
        }

        print("building + serving {} for the web (wasm)...\n", name);
        ProcCmd c = { .args = {
            minc, "run", "--target", "wasm", src,
            "-o", wasm_out
        } };
        if no_run { proc_arg(&c, "--no-browser"); }
        ProcResult r = proc_run(&c);
        rc = r.exit_code;
        proc_result_free(&r);
    } else {
        string exe = join_named("build", name, EXE_SUFFIX);
        defer free(exe);
        print("building {}\n", name);
        ProcCmd c = { .args = { minc, src, "-o", exe } };
        ProcResult r = proc_run(&c);
        rc = r.exit_code;
        proc_result_free(&r);
        if rc != 0 || !path_exists(exe) { die("minc compile failed"); }
        print("built {}\n", exe);
        if str_equal(verb, "run") && !no_run {
            ProcCmd run = { .args = { exe } };
            ProcResult rr = proc_run(&run);
            rc = rr.exit_code;
            proc_result_free(&rr);
        }
    }
    return rc;
}

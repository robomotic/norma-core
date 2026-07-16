use std::env;
use std::error::Error;
use std::path::{Path, PathBuf};

fn main() -> Result<(), Box<dyn Error>> {
    println!("cargo:rerun-if-env-changed=PKG_CONFIG_PATH");
    println!("cargo:rerun-if-env-changed=LIBCAMERA_LIB_DIR");
    println!("cargo:rerun-if-changed=wrapper.hpp");
    println!("cargo:rerun-if-changed=wrapper.cpp");

    let mut pkg_config = pkg_config::Config::new();
    pkg_config.cargo_metadata(false);

    let libcamera = pkg_config
        .probe("libcamera")
        .map_err(|e| build_error(format!("failed to find libcamera via pkg-config: {e}")))?;

    let target = env::var("TARGET").unwrap_or_default();
    let host = env::var("HOST").unwrap_or_default();
    // Any target/host mismatch is a cross-compile, not just building from a
    // non-Linux host (e.g. a Linux amd64 CI runner cross-compiling to Linux
    // arm64 is cross-compiling too, and must link the pre-built wrapper
    // rather than compiling wrapper.cpp against the checked-in .pc files,
    // whose Cflags bake in an absolute path from wherever they were built.
    let is_cross = target != host;

    // The checked-in .pc files bake in an absolute path from wherever the
    // vendored libcamera artifacts were originally built (e.g.
    // /Users/<someone>/...), which is wrong on any other machine. When
    // LIBCAMERA_LIB_DIR points at those vendored artifacts (set by the
    // Makefile/CI whenever the ov5647 feature is on), derive the include
    // dir from it instead of trusting pkg-config's Cflags. Falls back to
    // pkg-config's paths for a genuine native build against a system
    // libcamera-dev install (e.g. apt-installed on a Pi), where
    // LIBCAMERA_LIB_DIR won't be set.
    // libcamera's own meson.build installs public headers under
    // <includedir>/libcamera/libcamera/*.h (yes, "libcamera" twice), so the
    // -I path needs that extra segment for `#include <libcamera/foo.h>` to
    // resolve.
    let vendored_include_dir = env::var("LIBCAMERA_LIB_DIR").ok().and_then(|lib_dir| {
        Path::new(&lib_dir)
            .parent()
            .and_then(Path::parent)
            .map(|usr| usr.join("include").join("libcamera"))
    });
    let include_paths: Vec<PathBuf> = match &vendored_include_dir {
        Some(dir) => vec![dir.clone()],
        None => libcamera.include_paths.clone(),
    };

    if is_cross {
        let lib_dir = env::var("LIBCAMERA_LIB_DIR").map_err(|_| {
            build_error("cross-compiling requires LIBCAMERA_LIB_DIR with pre-built libcamera libs")
        })?;
        let wrapper_lib = Path::new(&lib_dir).join("libcamera_wrapper.a");

        if !wrapper_lib.exists() {
            return Err(build_error(format!(
                "pre-built wrapper not found at {}",
                wrapper_lib.display()
            )));
        }

        println!("cargo:rustc-link-search=native={lib_dir}");
        println!("cargo:rustc-link-lib=static=camera_wrapper");
    } else {
        let mut build = cc::Build::new();
        build
            .cpp(true)
            .std("c++17")
            .file("wrapper.cpp")
            .warnings(false);

        for path in &include_paths {
            build.include(path);
        }

        build.compile("camera_wrapper");
    }

    if vendored_include_dir.is_none() {
        // pkg-config's link_paths come from the same .pc Cflags/Libs as
        // include_paths above; when we're using the vendored artifacts the
        // real search path is added via LIBCAMERA_LIB_DIR below instead, so
        // skip these to avoid a bogus "unable to open library directory"
        // linker warning for the baked-in path.
        for path in &libcamera.link_paths {
            println!("cargo:rustc-link-search=native={}", path.display());
        }
    }
    if let Ok(lib_dir) = env::var("LIBCAMERA_LIB_DIR") {
        println!("cargo:rustc-link-search=native={lib_dir}");
        println!("cargo:rustc-link-arg=-Wl,-rpath-link,{lib_dir}");

        let libpisp_present = ["libpisp.so", "libpisp.so.1", "libpisp.so.1.3.0"]
            .iter()
            .any(|name| Path::new(&lib_dir).join(name).exists());
        if libpisp_present {
            println!("cargo:rustc-link-lib=dylib=pisp");
        }
    }
    for lib in &libcamera.libs {
        println!("cargo:rustc-link-lib={lib}");
    }

    let mut builder = bindgen::Builder::default()
        .header("wrapper.hpp")
        .allowlist_function("lc_.*")
        .allowlist_type("lc_.*")
        .allowlist_var("LC_.*")
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()));

    for path in &include_paths {
        builder = builder.clang_arg(format!("-I{}", path.display()));
    }

    let bindings = builder
        .generate()
        .map_err(|e| build_error(format!("failed to generate bindings: {e}")))?;
    let out_path = PathBuf::from(
        env::var("OUT_DIR").map_err(|e| build_error(format!("OUT_DIR is not set: {e}")))?,
    );

    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .map_err(|e| build_error(format!("failed to write bindings: {e}")))?;

    Ok(())
}

fn build_error(message: impl Into<String>) -> Box<dyn Error> {
    std::io::Error::other(message.into()).into()
}

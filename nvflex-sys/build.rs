use std::env;
use std::path::PathBuf;

fn main() {
    // tell cargo to invalidate the built crate whenever the wrapper.hpp file changes
    println!("cargo:rerun-if-changed=wrapper.hpp");

    let out_path = PathBuf::from(env::var("OUT_DIR").expect("OUT_DIR enviroment variable not set"));
    let bindings = bindgen::Builder::default()
        .header("wrapper.hpp")
        .allowlist_function("NvFlex.*")
        .allowlist_type("NvFlex.*")
        .allowlist_var("NvFlex.*")
        .allowlist_var("NV_FLEX_.*")
        .prepend_enum_name(false)
        .layout_tests(false)
        .derive_debug(true);

    #[cfg(feature = "ext")]
    let bindings = bindings.clang_arg("-DUSE_NV_EXT");

    let bindings = bindings
        .generate()
        .expect("unable to generate bindings")
        .write_to_file(out_path.join("bindings.rs"))
        .expect("unable to write bindings");

    // link libraries
    link()
}

macro_rules! link_lib {
	( $fmt:literal, $($x:tt),* ) => {
		println!("cargo:rustc-link-lib={}", format!($fmt, $($x),* ));
	}
}

#[cfg(debug_assertions)]
const VERSION: &str = "Debug";

#[cfg(not(debug_assertions))]
const VERSION: &str = "Release";

#[cfg(all(target_os = "windows"))]
fn link() {
    let ptr_width = env::var("CARGO_CFG_TARGET_POINTER_WIDTH")
        .expect("CARGO_CFG_TARGET_POINTER_WIDTH enviroment variable not set");

    let end = if ptr_width == "64" { "64" } else { "86" };

    let path = PathBuf::from(
        std::env::var("CARGO_MANIFEST_DIR")
            .expect("CARGO_MANIFEST_DIR enviroment variable not set"),
    )
    .join("FleX")
    .join("lib")
    .join(format!("win{}", end));

    println!("cargo:rustc-link-search=native={}", path.display());

    #[cfg(feature = "d3d")]
    {
        link_lib!("NvFlex{}D3D_x{}", VERSION, end);
        #[cfg(feature = "ext")]
        link_lib!("NvFlexExt{}D3D_x{}", VERSION, end);
    }

    #[cfg(feature = "cuda")]
    {
        link_lib!("NvFlex{}CUDA_x{}", VERSION, end);

        #[cfg(feature = "ext")]
        link_lib!("NvFlexExt{}CUDA_x{}", VERSION, end);
    }
}

#[cfg(all(target_os = "linux"))]
fn link() {
    let ptr_width = env::var("CARGO_CFG_TARGET_POINTER_WIDTH")
        .expect("CARGO_CFG_TARGET_POINTER_WIDTH enviroment variable not set");

    assert!(ptr_width == "64", "only 64-bit Linux is supported!");

    let path = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap())
        .join("FleX")
        .join("lib")
        .join("linux64");

    println!("cargo:rustc-link-search=native={}", path.display());

    #[cfg(feature = "d3d")]
    panic!("d3d is not supported on Linux, use the CUDA feature");

    #[cfg(feature = "cuda")]
    {
        link_lib!("NvFlex{}CUDA_x64", VERSION);

        #[cfg(feature = "ext")]
        link_lib!("NvFlexExt{}CUDA_x64", VERSION);
    }
}

#[cfg(all(target_os = "android"))]
fn link() {
    let path = PathBuf::from(std::env::var("CARGO_MANIFEST_DIR").unwrap())
        .join("FleX")
        .join("lib")
        .join("android");

    println!("cargo:rustc-link-search=native={}", path.display());

    #[cfg(feature = "d3d")]
    panic!("d3d is not supported on Android, use the CUDA feature");

    #[cfg(feature = "cuda")]
    {
        link_lib!("libNvFlex{}CUDA_aarch64", VERSION);

        #[cfg(feature = "ext")]
        link_lib!("libNvFlexExt{}CUDA_aarch64", VERSION);
    }
}

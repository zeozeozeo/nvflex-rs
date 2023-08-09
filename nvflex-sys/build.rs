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

    bindings
        .generate()
        .expect("unable to generate bindings")
        .write_to_file(out_path.join("bindings.rs"))
        .expect("unable to write bindings");

    // link libraries
    link();

    // if we're on Windows, copy the DLL files into the target directory
    #[cfg(target_os = "windows")]
    copy_dlls_to_target_directory();
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

#[cfg(target_os = "windows")]
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

#[cfg(target_os = "linux")]
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

#[cfg(target_os = "android")]
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

#[cfg(target_os = "windows")]
fn copy_dlls_to_target_directory() {
    // we need to get to the target/debug or the target/release directory
    // we can use the `OUT_DIR` envvar which points to target/debug/build/nvflex-sys.*./out,
    // so we need to pop the last 3 entries of the path
    let mut out_dir =
        PathBuf::from(std::env::var("OUT_DIR").expect("OUT_DIR enviroment variable not set"));
    out_dir.pop(); // pop `/out`
    out_dir.pop(); // pop `/nvflex-sys.*.`
    out_dir.pop(); // pop `/build`

    let ptr_width = env::var("CARGO_CFG_TARGET_POINTER_WIDTH")
        .expect("CARGO_CFG_TARGET_POINTER_WIDTH enviroment variable not set");

    #[cfg(debug_assertions)]
    const DEBUG_BUILD: bool = true;

    #[cfg(not(debug_assertions))]
    const DEBUG_BUILD: bool = false;

    let flex_libs: Vec<&str> = if ptr_width == "64" {
        // win64
        vec![
            "amd_ags_x64.dll",
            "cudart64_92.dll",
            "GFSDK_Aftermath_Lib.x64.dll",
            if DEBUG_BUILD {
                "NvFlexDebugCUDA_x64.dll"
            } else {
                "NvFlexReleaseCUDA_x64.dll"
            },
            if DEBUG_BUILD {
                "NvFlexDebugD3D_x64.dll"
            } else {
                "NvFlexReleaseD3D_x64.dll"
            },
            if DEBUG_BUILD {
                "NvFlexDeviceDebug_x64.dll"
            } else {
                "NvFlexDeviceRelease_x64.dll"
            },
            if DEBUG_BUILD {
                "NvFlexExtDebugCUDA_x64.dll"
            } else {
                "NvFlexExtReleaseCUDA_x64.dll"
            },
            if DEBUG_BUILD {
                "NvFlexExtDebugD3D_x64.dll"
            } else {
                "NvFlexExtReleaseD3D_x64.dll"
            },
            "nvToolsExt64_1.dll",
        ]
    } else {
        // win32
        vec![
            "amd_ags_x86.dll",
            "cudart32_92.dll",
            "GFSDK_Aftermath_Lib.x86.dll",
            if DEBUG_BUILD {
                "NvFlexDebugCUDA_x86.dll"
            } else {
                "NvFlexReleaseCUDA_x86.dll"
            },
            if DEBUG_BUILD {
                "NvFlexDebugD3D_x86.dll"
            } else {
                "NvFlexReleaseD3D_x86.dll"
            },
            if DEBUG_BUILD {
                "NvFlexDeviceDebug_x86.dll"
            } else {
                "NvFlexDeviceRelease_x86.dll"
            },
            if DEBUG_BUILD {
                "NvFlexExtDebugCUDA_x86.dll"
            } else {
                "NvFlexExtReleaseCUDA_x86.dll"
            },
            if DEBUG_BUILD {
                "NvFlexExtDebugD3D_x86.dll"
            } else {
                "NvFlexExtReleaseD3D_x86.dll"
            },
            "nvToolsExt32_1.dll",
        ]
    };

    for lib in flex_libs {
        let mut dest_path = out_dir.clone();
        dest_path.push(lib);
        if dest_path.exists() {
            continue; // no need to copy this dll, it is already present
        }

        let mut src_path =
            PathBuf::from("FleX/bin/win".to_string() + if ptr_width == "64" { "64" } else { "32" });
        src_path.push(lib);

        // copy the library dll
        let _ = std::fs::copy(src_path, dest_path);
    }
}

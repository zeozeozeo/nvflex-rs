use std::{fs::File, io::Write, path::PathBuf};

// win32/win64 libraries that only have release versions
cfg_if::cfg_if! {
    if #[cfg(all(windows, target_arch = "x86"))] {
        // win32 libs, debug build
        /// amd_ags_x86.dll
        pub const AMD_AGS: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/amd_ags_x86.dll"),
            "amd_ags_x86.dll",
        );
        /// cudart32_92.dll
        pub const CUDART: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/cudart32_92.dll"),
            "cudart32_92.dll",
        );
        /// GFSDK_Aftermath_Lib.x86.dll
        pub const GFSDK_AFTERMATH_LIB: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/GFSDK_Aftermath_Lib.x86.dll"),
            "GFSDK_Aftermath_Lib.x86.dll",
        );
        /// nvToolsExt32_1.dll
        pub const NV_TOOLS_EXT: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/nvToolsExt32_1.dll"),
            "nvToolsExt32_1.dll",
        );
    } else if #[cfg(all(windows, target_arch = "x86_64"))] {
        // win64 libs
        /// amd_ags_x64.dll
        pub const AMD_AGS: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/amd_ags_x64.dll"),
            "amd_ags_x64.dll",
        );
        /// cudart64_92.dll
        pub const CUDART: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/cudart64_92.dll"),
            "cudart64_92.dll",
        );
        /// GFSDK_Aftermath_Lib.x64.dll
        pub const GFSDK_AFTERMATH_LIB: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/GFSDK_Aftermath_Lib.x64.dll"),
            "GFSDK_Aftermath_Lib.x64.dll",
        );
        /// nvToolsExt64_1.dll
        pub const NV_TOOLS_EXT: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/nvToolsExt64_1.dll"),
            "nvToolsExt64_1.dll",
        );
    }
}

// win32 libraries that have release and debug versions
cfg_if::cfg_if! {
    if #[cfg(all(windows, target_arch = "x86", debug_assertions))] {
        // win32, debug build
        /// NvFlexDebugCUDA_x86.dll
        pub const NV_FLEX_DEBUG_CUDA: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/NvFlexDebugCUDA_x86.dll"),
            "NvFlexDebugCUDA_x86.dll",
        );
        /// NvFlexDebugD3D_x86.dll
        pub const NV_FLEX_DEBUG_D3D: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/NvFlexDebugD3D_x86.dll"),
            "NvFlexDebugD3D_x86.dll",
        );
        /// NvFlexDeviceDebug_x86.dll
        pub const NV_FLEX_DEVICE_DEBUG: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/NvFlexDeviceDebug_x86.dll"),
            "NvFlexDeviceDebug_x86.dll",
        );
        /// NvFlexExtDebugCUDA_x86.dll
        pub const NV_FLEX_EXT_DEBUG_CUDA: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/NvFlexExtDebugCUDA_x86.dll"),
            "NvFlexExtDebugCUDA_x86.dll",
        );
        /// NvFlexExtDebugD3D_x86.dll
        pub const NV_FLEX_EXT_DEBUG_D3D: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/NvFlexExtDebugD3D_x86.dll"),
            "NvFlexExtDebugD3D_x86.dll",
        );
    } else if #[cfg(all(windows, target_arch = "x86", not(debug_assertions)))] {
        // win32, release build
        /// NvFlexDeviceRelease_x86.dll
        pub const NV_FLEX_DEVICE_RELEASE: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/NvFlexDeviceRelease_x86.dll"),
            "NvFlexDeviceRelease_x86.dll",
        );
        /// NvFlexExtReleaseCUDA_x86.dll
        pub const NV_FLEX_EXT_RELEASE_CUDA: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/NvFlexExtReleaseCUDA_x86.dll"),
            "NvFlexExtReleaseCUDA_x86.dll",
        );
        /// NvFlexExtReleaseD3D_x86.dll
        pub const NV_FLEX_EXT_RELEASE_D3D: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/NvFlexExtReleaseD3D_x86.dll"),
            "NvFlexExtReleaseD3D_x86.dll",
        );
        /// NvFlexReleaseCUDA_x86.dll
        pub const NV_FLEX_RELEASE_CUDA: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/NvFlexReleaseCUDA_x86.dll"),
            "NvFlexReleaseCUDA_x86.dll",
        );
        /// NvFlexReleaseD3D_x86.dll
        pub const NV_FLEX_RELEASE_D3D: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win32/NvFlexReleaseD3D_x86.dll"),
            "NvFlexReleaseD3D_x86.dll",
        );
    }
}

// win64 libraries that have release and debug versions
cfg_if::cfg_if! {
    if #[cfg(all(windows, target_arch = "x86_64", debug_assertions))] {
        // win64, debug build
        /// NvFlexDebugCUDA_x64.dll
        pub const NV_FLEX_DEBUG_CUDA: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/NvFlexDebugCUDA_x64.dll"),
            "NvFlexDebugCUDA_x64.dll",
        );
        /// NvFlexDebugD3D_x64.dll
        pub const NV_FLEX_DEBUG_D3D: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/NvFlexDebugD3D_x64.dll"),
            "NvFlexDebugD3D_x64.dll",
        );
        /// NvFlexDeviceDebug_x64.dll
        pub const NV_FLEX_DEVICE_DEBUG: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/NvFlexDeviceDebug_x64.dll"),
            "NvFlexDeviceDebug_x64.dll",
        );
        /// NvFlexExtDebugCUDA_x64.dll
        pub const NV_FLEX_EXT_DEBUG_CUDA: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/NvFlexExtDebugCUDA_x64.dll"),
            "NvFlexExtDebugCUDA_x64.dll",
        );
        /// NvFlexExtDebugD3D_x64.dll
        pub const NV_FLEX_EXT_DEBUG_D3D: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/NvFlexExtDebugD3D_x64.dll"),
            "NvFlexExtDebugD3D_x64.dll",
        );
    } else if #[cfg(all(windows, target_arch = "x86_64", not(debug_assertions)))] {
        // win64, release build
        /// NvFlexDeviceRelease_x64.dll
        pub const NV_FLEX_DEVICE_RELEASE: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/NvFlexDeviceRelease_x64.dll"),
            "NvFlexDeviceRelease_x64.dll",
        );
        /// NvFlexExtReleaseCUDA_x64.dll
        pub const NV_FLEX_EXT_RELEASE_CUDA: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/NvFlexExtReleaseCUDA_x64.dll"),
            "NvFlexExtReleaseCUDA_x64.dll",
        );
        /// NvFlexExtReleaseD3D_x64.dll
        pub const NV_FLEX_EXT_RELEASE_D3D: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/NvFlexExtReleaseD3D_x64.dll"),
            "NvFlexExtReleaseD3D_x64.dll",
        );
        /// NvFlexReleaseCUDA_x64.dll
        pub const NV_FLEX_RELEASE_CUDA: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/NvFlexReleaseCUDA_x64.dll"),
            "NvFlexReleaseCUDA_x64.dll",
        );
        /// NvFlexReleaseD3D_x64.dll
        pub const NV_FLEX_RELEASE_D3D: (&[u8], &str) = (
            include_bytes!("../FleX/bin/win64/NvFlexReleaseD3D_x64.dll"),
            "NvFlexReleaseD3D_x64.dll",
        );
    }
}

/// Returns the binary data and the library filenames.
///
/// If the `embed_libs` feature is disabled, will return an empty vector.
///
/// If building in debug mode, only debug libraries are embedded, same for the release mode.
pub fn get_libraries() -> Vec<(&'static [u8], &'static str)> {
    #[cfg(feature = "embed_libs")]
    {
        #[cfg(debug_assertions)]
        return vec![
            AMD_AGS,
            CUDART,
            GFSDK_AFTERMATH_LIB,
            NV_FLEX_DEBUG_CUDA,
            NV_FLEX_DEBUG_D3D,
            NV_FLEX_DEVICE_DEBUG,
            NV_FLEX_EXT_DEBUG_CUDA,
            NV_FLEX_EXT_DEBUG_D3D,
            NV_TOOLS_EXT,
        ];
        #[cfg(not(debug_assertions))]
        return vec![
            AMD_AGS,
            CUDART,
            GFSDK_AFTERMATH_LIB,
            NV_FLEX_DEVICE_RELEASE,
            NV_FLEX_EXT_RELEASE_CUDA,
            NV_FLEX_EXT_RELEASE_D3D,
            NV_FLEX_RELEASE_CUDA,
            NV_FLEX_RELEASE_D3D,
            NV_TOOLS_EXT,
        ];
    }
    #[cfg(not(feature = "embed_libs"))]
    {
        return vec![];
    }
}

/// Write embedded libraries to disk. Can be used as a hacky way of embedding the FleX libraries inside the executable.
///
/// If the `embed_libs` feature is disabled, will do nothing.
///
/// TODO: maybe we can somehow tell FleX to load libraries from memory instead of loading them from disk?
///
/// # Arguments
///
/// * `path` - Directory where the files should be created.
/// * `overwrite` - If true, existing library files will be overwritten.
///
/// # Example
///
/// ```
/// // write all required libraries to the current executable path
/// let mut executable_dir = std::env::current_exe().unwrap();
/// executable_dir.pop(); // pop the executable name
/// nvflex_sys::embed::write_libraries_to_path(executable_dir, true).unwrap();
/// ```
pub fn write_libraries_to_path(path: PathBuf, overwrite: bool) -> Result<(), std::io::Error> {
    for (data, filename) in get_libraries() {
        let mut lib_path = path.clone();
        lib_path.push(filename);
        if lib_path.try_exists()? && !overwrite {
            continue;
        }

        let mut f = File::create(lib_path)?;
        f.write_all(data)?;
    }
    Ok(())
}

/// Write embedded libraries to the current executable directory. Can be used as a hacky way of embedding the FleX libraries inside the executable.
///
/// If the `embed_libs` feature is disabled, will do nothing.
///
/// # Arguments
///
/// * `overwrite` - If true, existing library files will be overwritten.
#[inline]
pub fn write_libraries_to_exe_dir(overwrite: bool) -> Result<(), std::io::Error> {
    write_libraries_to_path(std::env::current_exe()?, overwrite)
}

/// Whether all required dynamic libraries are present inside the given directory. If [`None`], there was an error getting the file info.
///
/// If the `embed_libs` feature is disabled, will do nothing.
///
/// # Arguments
///
/// * `path` - Directory to search for libraries.
pub fn libraries_present(path: PathBuf) -> Option<bool> {
    for (_, filename) in get_libraries() {
        let mut lib_path = path.clone();
        lib_path.push(filename);
        if !lib_path.try_exists().ok()? {
            return Some(false); // this library does not exist
        }
    }
    Some(true)
}

/// Whether all required dynamic libraries are present inside the executable. If [`None`], there was an error getting the file info.
///
/// If the `embed_libs` feature is disabled, will do nothing.
#[inline]
pub fn libraries_present_in_exe_dir() -> Option<bool> {
    libraries_present(std::env::current_exe().ok()?)
}

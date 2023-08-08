// win32 libs
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const AMD_AGS: &[u8] = include_bytes!("../FleX/bin/win32/amd_ags_x86.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const CUDART: &[u8] = include_bytes!("../FleX/bin/win32/cudart32_92.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const GFSDK_AFTERMATH_LIB: &[u8] =
    include_bytes!("../FleX/bin/win32/GFSDK_Aftermath_Lib.x86.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const NV_FLEX_DEBUG_CUDA: &[u8] = include_bytes!("../FleX/bin/win32/NvFlexDebugCUDA_x86.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const NV_FLEX_DEBUG_D3D: &[u8] = include_bytes!("../FleX/bin/win32/NvFlexDebugD3D_x86.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const NV_FLEX_DEVICE_DEBUG: &[u8] =
    include_bytes!("../FleX/bin/win32/NvFlexDeviceDebug_x86.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const NV_FLEX_DEVICE_RELEASE: &[u8] =
    include_bytes!("../FleX/bin/win32/NvFlexDeviceRelease_x86.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const NV_FLEX_EXT_DEBUG_CUDA: &[u8] =
    include_bytes!("../FleX/bin/win32/NvFlexExtDebugCUDA_x86.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const NV_FLEX_EXT_DEBUG_D3D: &[u8] =
    include_bytes!("../FleX/bin/win32/NvFlexExtDebugD3D_x86.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const NV_FLEX_EXT_RELEASE_CUDA: &[u8] =
    include_bytes!("../FleX/bin/win32/NvFlexExtReleaseCUDA_x86.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const NV_FLEX_EXT_RELEASE_D3D: &[u8] =
    include_bytes!("../FleX/bin/win32/NvFlexExtReleaseD3D_x86.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const NV_FLEX_RELEASE_CUDA: &[u8] =
    include_bytes!("../FleX/bin/win32/NvFlexReleaseCUDA_x86.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const NV_FLEX_RELEASE_D3D: &[u8] =
    include_bytes!("../FleX/bin/win32/NvFlexReleaseD3D_x86.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86", feature = "embed_libs"))]
pub const NV_TOOLS_EXT: &[u8] = include_bytes!("../FleX/bin/win32/nvToolsExt32_1.dll");

// win64 libs
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const AMD_AGS: &[u8] = include_bytes!("../FleX/bin/win64/amd_ags_x64.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const CUDART: &[u8] = include_bytes!("../FleX/bin/win64/cudart64_92.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const GFSDK_AFTERMATH_LIB: &[u8] =
    include_bytes!("../FleX/bin/win64/GFSDK_Aftermath_Lib.x64.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const NV_FLEX_DEBUG_CUDA: &[u8] = include_bytes!("../FleX/bin/win64/NvFlexDebugCUDA_x64.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const NV_FLEX_DEBUG_D3D: &[u8] = include_bytes!("../FleX/bin/win64/NvFlexDebugD3D_x64.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const NV_FLEX_DEVICE_DEBUG: &[u8] =
    include_bytes!("../FleX/bin/win64/NvFlexDeviceDebug_x64.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const NV_FLEX_DEVICE_RELEASE: &[u8] =
    include_bytes!("../FleX/bin/win64/NvFlexDeviceRelease_x64.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const NV_FLEX_EXT_DEBUG_CUDA: &[u8] =
    include_bytes!("../FleX/bin/win64/NvFlexExtDebugCUDA_x64.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const NV_FLEX_EXT_DEBUG_D3D: &[u8] =
    include_bytes!("../FleX/bin/win64/NvFlexExtDebugD3D_x64.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const NV_FLEX_EXT_RELEASE_CUDA: &[u8] =
    include_bytes!("../FleX/bin/win64/NvFlexExtReleaseCUDA_x64.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const NV_FLEX_EXT_RELEASE_D3D: &[u8] =
    include_bytes!("../FleX/bin/win64/NvFlexExtReleaseD3D_x64.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const NV_FLEX_RELEASE_CUDA: &[u8] =
    include_bytes!("../FleX/bin/win64/NvFlexReleaseCUDA_x64.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const NV_FLEX_RELEASE_D3D: &[u8] =
    include_bytes!("../FleX/bin/win64/NvFlexReleaseD3D_x64.dll ");
#[cfg(all(target_os = "windows", target_arch = "x86_64", feature = "embed_libs"))]
pub const NV_TOOLS_EXT: &[u8] = include_bytes!("../FleX/bin/win64/nvToolsExt64_1.dll");

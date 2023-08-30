use std::ffi::c_void;

use nvflex_sys::*;

use crate::{
    rstr, ConvexMeshId, DistanceFieldId, ErrorCallback, InitDesc, Solver, SolverDesc,
    TriangleMeshId, VERSION,
};

/// Opaque type representing a library that can create FlexSolvers, FlexTriangleMeshes, and NvFlexBuffers
#[derive(Debug)]
pub struct Library {
    /// Pointer to the underlying [`NvFlexLibrary`]. This should never escape from the struct.
    pub(crate) lib: *mut NvFlexLibrary,
}

impl Library {
    /// Initialize library, should be called before any other API function.
    ///
    /// # Parameters
    ///
    /// * `version` - The version number the app is expecting, should almost always be [`VERSION`]
    ///
    /// * `error_func` - (Optional) The callback used for reporting errors.
    ///
    /// * `desc` - (Optional) The [`InitDesc`] struct defining the device ordinal, D3D device/context and the type of D3D compute being used
    ///
    /// # Returns
    ///
    /// A library instance that can be used to allocate shared object such as triangle meshes, buffers, etc
    #[inline]
    pub fn new(error_func: ErrorCallback, desc: Option<InitDesc>) -> Option<Self> {
        unsafe {
            let lib = NvFlexInit(
                VERSION as _,
                error_func,
                if let Some(desc) = desc {
                    std::mem::transmute(&desc)
                } else {
                    std::ptr::null_mut()
                },
            );
            if lib.is_null() {
                None
            } else {
                Some(Self { lib })
            }
        }
    }

    /// Get the list of active solvers in the library
    ///
    /// If the size of the array is smaller than the number of active solvers, only the first n entries are copied.
    ///
    /// # Parameters
    ///
    /// * `solvers` - Pointer to array
    ///
    /// # Returns
    ///
    /// The number of active solvers in the library
    #[inline]
    pub fn get_solvers(&self, solvers: &mut [*mut Solver]) -> i32 {
        unsafe { NvFlexGetSolvers(self.lib, solvers.as_mut_ptr() as *mut _, solvers.len() as _) }
    }

    /// Get the list of triangle mesh ids in the library
    ///
    /// If the size of the array is smaller than the number of triangle mesh ids, only the first n entries are copied.
    ///
    /// # Parameters
    ///
    /// * `meshes` - Pointer to array
    ///
    /// # Returns
    ///
    /// The number of triangle mesh ids in the library
    #[inline]
    pub fn get_triangle_mesh_ids(&self, meshes: &mut [TriangleMeshId]) -> i32 {
        unsafe {
            NvFlexGetTriangleMeshes(self.lib, meshes.as_mut_ptr() as *mut _, meshes.len() as _)
        }
    }

    /// Get the list of signed distance fields in the library
    ///
    /// If the size of the array is smaller than the number of signed distance fields, only the first n entries are copied.
    ///
    /// # Parameters
    ///
    /// * `sdfs` - Pointer to array
    ///
    /// # Returns
    ///
    /// The number of signed distance fields in the library
    #[inline]
    pub fn get_distance_field_ids(&self, sdfs: &mut [DistanceFieldId]) -> i32 {
        unsafe { NvFlexGetDistanceFields(self.lib, sdfs.as_mut_ptr() as *mut _, sdfs.len() as _) }
    }

    /// Get the list of convex meshes in the library
    ///
    /// If the size of the array is smaller than the number of convex meshes, only the first n entries are copied.
    ///
    /// # Parameters
    ///
    /// * `meshes` - Pointer to array
    ///
    /// # Returns
    ///
    /// The number of convex meshes in the library
    #[inline]
    pub fn get_convex_mesh_ids(&self, meshes: &mut [ConvexMeshId]) -> i32 {
        unsafe { NvFlexGetConvexMeshes(self.lib, meshes.as_mut_ptr() as *mut _, meshes.len() as _) }
    }

    /// Ensures that the CUDA context the library was initialized with is present on the current thread
    #[inline]
    pub fn acquire_context(&self) {
        unsafe { NvFlexAcquireContext(self.lib) };
    }

    /// Restores the CUDA context (if any) that was present on the last call to `acquire_context()`
    ///
    /// Note: the acquire/restore pair of calls must come from the same thread
    #[inline]
    pub fn restore_context(&self) {
        unsafe { NvFlexRestoreContext(self.lib) };
    }

    /// Returns a string with the compute device name
    #[inline]
    pub fn get_device_name(&self) -> &str {
        rstr!(NvFlexGetDeviceName(self.lib))
    }

    /// Retrieve the device and context for the the library.
    ///
    /// On CUDA the context pointer will be filled with a pointer to a `CUcontext` structure
    ///
    /// On D3D the device and context pointers will be filled with pointers to a `NvFlex::Device`, and `NvFlex::Context` wrapper
    ///
    /// # Returns
    ///
    /// * .0 - Pointer to a device pointer, see description
    /// * .1 - Pointer to a context pointer, see description
    #[inline]
    pub fn get_device_and_context(&self) -> (*mut c_void, *mut c_void) {
        unsafe {
            let mut device = std::ptr::null_mut();
            let mut context = std::ptr::null_mut();
            NvFlexGetDeviceAndContext(self.lib, &mut device, &mut context);
            (device, context)
        }
    }

    /// Force a pipeline flush to ensure any queued work is submitted to the GPU
    #[inline]
    pub fn flush(&self) {
        unsafe { NvFlexFlush(self.lib) };
    }

    // (no documentation)
    #[inline]
    pub fn wait(&self) {
        unsafe { NvFlexWait(self.lib) };
    }

    /// Debug method (unsupported)
    #[inline]
    pub fn compute_wait_for_graphics(&self) {
        unsafe { NvFlexComputeWaitForGraphics(self.lib) };
    }

    /// Debug method (unsupported)
    #[inline]
    pub fn get_data_aftermath(&self, p_data_out: *mut c_void, p_status_out: *mut c_void) {
        unsafe { NvFlexGetDataAftermath(self.lib, p_data_out, p_status_out) };
    }

    /// Shorthand for `Solver::new(lib, desc)`
    #[inline]
    pub fn new_solver(self, desc: &SolverDesc) -> Solver {
        Solver::new(self, desc)
    }
}

impl Drop for Library {
    #[inline]
    fn drop(&mut self) {
        unsafe {
            NvFlexShutdown(self.lib);
        }
    }
}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Send for Library {}

#[cfg(feature = "unsafe_send_sync")]
unsafe impl Sync for Library {}

/// Get library version number
#[inline]
pub fn get_version() -> i32 {
    unsafe { NvFlexGetVersion() }
}

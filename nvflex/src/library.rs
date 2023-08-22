use nvflex_sys::*;

use crate::{ErrorCallback, InitDesc, Solver, VERSION};

/// Opaque type representing a library that can create FlexSolvers, FlexTriangleMeshes, and NvFlexBuffers
#[derive(Debug)]
pub struct Library {
    pub(crate) lib: *mut NvFlexLibrary,
}

impl Library {
    /// Initialize library, should be called before any other API function.
    ///
    /// # Parameters
    ///
    /// * `version` - The version number the app is expecting, should almost always be [`VERSION`]
    ///
    /// * `errorFunc` - The callback used for reporting errors.
    ///
    /// * `desc` - The [`InitDesc`] struct defining the device ordinal, D3D device/context and the type of D3D compute being used
    ///
    /// # Returns
    ///
    /// A library instance that can be used to allocate shared object such as triangle meshes, buffers, etc
    #[inline]
    pub fn init(error_func: ErrorCallback, desc: &mut InitDesc) -> Self {
        unsafe {
            Self {
                lib: NvFlexInit(VERSION as _, error_func, desc as *mut _ as *mut _),
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
    /// * `n` - Size of array
    ///
    /// # Returns
    ///
    /// The number of active solvers in the library
    #[inline]
    pub fn get_solvers(&self, solvers: *mut *mut Solver, n: i32) -> i32 {
        unsafe { NvFlexGetSolvers(self.lib, solvers as *mut *mut _, n) }
    }

    /// Get a vector of active solvers in the library (util)
    #[inline]
    pub fn get_solvers_vec(&self) -> Vec<Solver> {
        let num_solvers = self.get_solvers(std::ptr::null_mut(), 0);
        let mut solvers: Vec<Solver> = Vec::with_capacity(num_solvers as _);
        self.get_solvers(solvers.as_mut_ptr() as *mut *mut _, num_solvers);
        solvers
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

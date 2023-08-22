use std::ffi::c_void;

use nvflex_sys::*;

use crate::{
    rstr, ConvexMeshId, DistanceFieldId, ErrorCallback, InitDesc, Solver, TriangleMeshId, VERSION,
};

/// Opaque type representing a library that can create FlexSolvers, FlexTriangleMeshes, and NvFlexBuffers
#[derive(Debug, Clone)]
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
        unsafe {
            let num_solvers = self.get_solvers(std::ptr::null_mut(), 0);
            let mut solvers: Vec<Solver> = vec![std::mem::zeroed::<Solver>(); num_solvers as _];
            self.get_solvers(solvers.as_mut_ptr() as *mut *mut _, num_solvers);
            solvers
        }
    }

    /// Get the list of triangle mesh ids in the library
    ///
    /// If the size of the array is smaller than the number of triangle mesh ids, only the first n entries are copied.
    ///
    /// # Parameters
    ///
    /// * `meshes` - Pointer to array
    /// * `n` - Size of array
    ///
    /// # Returns
    ///
    /// The number of triangle mesh ids in the library
    #[inline]
    pub fn get_triangle_mesh_ids(&self, meshes: *mut TriangleMeshId, n: i32) -> i32 {
        unsafe { NvFlexGetTriangleMeshes(self.lib, meshes as *mut _, n) }
    }

    /// Get a vector of triangle mesh ids in the library (util)
    #[inline]
    pub fn get_triangle_mesh_ids_vec(&self) -> Vec<TriangleMeshId> {
        let num_meshes = self.get_triangle_mesh_ids(std::ptr::null_mut(), 0);
        let mut meshes: Vec<TriangleMeshId> = vec![0; num_meshes as _];
        self.get_triangle_mesh_ids(meshes.as_mut_ptr(), num_meshes);
        meshes
    }

    /// Get the list of signed distance fields in the library
    ///
    /// If the size of the array is smaller than the number of signed distance fields, only the first n entries are copied.
    ///
    /// # Parameters
    ///
    /// * `sdfs` - Pointer to array
    /// * `n` - Size of array
    ///
    /// # Returns
    ///
    /// The number of signed distance fields in the library
    #[inline]
    pub fn get_distance_field_ids(&self, sdfs: *mut DistanceFieldId, n: i32) -> i32 {
        unsafe { NvFlexGetDistanceFields(self.lib, sdfs as *mut _, n) }
    }

    /// Get a vector of signed distance field ids in the library (util)
    #[inline]
    pub fn get_distance_field_ids_vec(&self) -> Vec<DistanceFieldId> {
        let num_distance_fields = self.get_distance_field_ids(std::ptr::null_mut(), 0);
        let mut distance_fields: Vec<DistanceFieldId> = vec![0; num_distance_fields as _];
        self.get_distance_field_ids(distance_fields.as_mut_ptr(), num_distance_fields);
        distance_fields
    }

    /// Get the list of convex meshes in the library
    ///
    /// If the size of the array is smaller than the number of convex meshes, only the first n entries are copied.
    ///
    /// # Parameters
    ///
    /// * `meshes` - Pointer to array
    /// * `n` - Size of array
    ///
    /// # Returns
    ///
    /// The number of convex meshes in the library
    #[inline]
    pub fn get_convex_mesh_ids(&self, meshes: *mut ConvexMeshId, n: i32) -> i32 {
        unsafe { NvFlexGetConvexMeshes(self.lib, meshes as *mut _, n) }
    }

    /// Get a vector of convex mesh ids in the library (util)
    #[inline]
    pub fn get_convex_mesh_ids_vec(&self) -> Vec<ConvexMeshId> {
        let num_convex_meshes = self.get_convex_mesh_ids(std::ptr::null_mut(), 0);
        let mut convex_meshes: Vec<ConvexMeshId> = vec![0; num_convex_meshes as _];
        self.get_convex_mesh_ids(convex_meshes.as_mut_ptr(), num_convex_meshes);
        convex_meshes
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
